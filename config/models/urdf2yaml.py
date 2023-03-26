import argparse
import yaml
import xml.etree.ElementTree as ET

def urdf_to_yaml(urdf_file_path, yaml_file_path):
    # Load URDF file as XML
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()

    # Create dictionary for YAML output
    yaml_dict = {}

    # Loop through each element in the URDF file
    for elem in root.iter():
        # Ignore comments and whitespace
        if elem.tag is ET.Comment or not elem.tag.strip():
            continue

        # Add element attributes to YAML dictionary
        if elem.attrib:
            if elem.tag not in yaml_dict:
                yaml_dict[elem.tag] = []

            yaml_dict[elem.tag].append(elem.attrib)

        # Add element text to YAML dictionary
        if elem.text and elem.text.strip():
            if elem.tag not in yaml_dict:
                yaml_dict[elem.tag] = []

            yaml_dict[elem.tag].append(elem.text.strip())

    # Write YAML file
    with open(yaml_file_path, 'w') as f:
        yaml.dump(yaml_dict, f)

if __name__ == '__main__':
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Convert URDF file to YAML')
    parser.add_argument('urdf_file', help='path to URDF file')
    parser.add_argument('yaml_file', help='path to output YAML file')
    args = parser.parse_args()

    # Convert URDF to YAML
    urdf_to_yaml(args.urdf_file, args.yaml_file)
