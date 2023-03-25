import argparse
import yaml
import xml.etree.ElementTree as ET

def urdf_to_yaml(urdf_file, yaml_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    urdf_dict = {"robot": {}}

    # Add robot name and version
    urdf_dict["robot"]["name"] = root.attrib["name"]
    if "version" in root.attrib:
        urdf_dict["robot"]["version"] = root.attrib["version"]

    # Add robot attributes
    for key, value in root.attrib.items():
        if key not in ["name", "version"]:
            urdf_dict["robot"][key] = value

    # Add robot links
    for link in root.findall(".//link"):
        link_dict = {}
        link_dict["name"] = link.attrib["name"]
        link_dict["visual"] = {}
        link_dict["collision"] = {}

        # Add link visuals
        visual = link.find(".//visual")
        if visual is not None:
            link_dict["visual"]["origin"] = visual.find("origin").attrib
            link_dict["visual"]["geometry"] = visual.find("geometry").attrib
            if visual.find("material") is not None:
                link_dict["visual"]["material"] = visual.find("material").attrib

        # Add link collisions
        collision = link.find(".//collision")
        if collision is not None:
            link_dict["collision"]["origin"] = collision.find("origin").attrib
            link_dict["collision"]["geometry"] = collision.find("geometry").attrib

        urdf_dict["robot"][link_dict["name"]] = link_dict

    # Add robot joints
    for joint in root.findall(".//joint"):
        joint_dict = {}
        joint_dict["name"] = joint.attrib["name"]
        joint_dict["type"] = joint.attrib["type"]
        joint_dict["parent"] = joint.find("parent").attrib["link"]
        joint_dict["child"] = joint.find("child").attrib["link"]
        joint_dict["origin"] = joint.find("origin").attrib
        if joint_dict["type"] != "fixed":
            joint_dict["axis"] = joint.find(".//axis").attrib
            if joint.find(".//limit") is not None:
                joint_dict["limit"] = joint.find(".//limit").attrib

        urdf_dict["robot"][joint_dict["name"]] = joint_dict

    # Write dictionary to YAML file
    with open(yaml_file, "w") as file:
        yaml.dump(urdf_dict, file)

if __name__ == '__main__':
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Convert URDF file to YAML')
    parser.add_argument('urdf_file', help='path to URDF file')
    parser.add_argument('yaml_file', help='path to output YAML file')
    args = parser.parse_args()

    # Convert URDF to YAML
    urdf_to_yaml(args.urdf_file, args.yaml_file)
