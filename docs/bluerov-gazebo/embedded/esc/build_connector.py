from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2] / ".local" / "ardupilot_gazebo" / "src" / "ArduPilotPlugin.cc"
OUTPUT = Path(__file__).parent / "build" / "ArduPilotPlugin.cc"

def replace_once(source, before, after):
    if source.count(before) != 1:
        raise ValueError(f"connector source changed; expected one occurrence of {before[:60]!r}")
    
    return source.replace(before, after, 1)

def prepare():
    source = SOURCE.read_text()
    source = replace_once(source, "#include <gz/msgs/int32.pb.h>",
                          "#include <gz/msgs/int32.pb.h>\n#include <gz/msgs/boolean.pb.h>")
    
    source = replace_once(source,
                          "  public: gz::transport::Node::Publisher pub;",
                          "  public: gz::transport::Node::Publisher pub;\n"
                          "  public: gz::transport::Node::Publisher validPub;")
    
    source = replace_once(source,
                          "          node.Advertise<msgs::Int32>(control.cmdTopic);",
                          "          node.Advertise<msgs::Int32>(control.cmdTopic);\n"
                          "      control.validPub = this->dataPtr->node.Advertise<msgs::Boolean>(\n"
                          "          control.cmdTopic + \"/source_valid\");")
    
    source = replace_once(source,
                          "  {\n    // skip servo output while not ready\n"
                          "    if (!this->dataPtr->controls[i].outputReady)",
                          "  {\n    if (this->dataPtr->controls[i].type == \"COMMAND\")\n"
                          "    {\n      msgs::Boolean valid;\n"
                          "      valid.set_data(this->dataPtr->controls[i].outputReady);\n"
                          "      this->dataPtr->controls[i].validPub.Publish(valid);\n    }\n"
                          "    // skip servo output while not ready\n"
                          "    if (!this->dataPtr->controls[i].outputReady)")
    
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(source)

    return OUTPUT

if __name__ == "__main__":
    print(prepare())