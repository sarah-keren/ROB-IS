import yaml
import os

generated_file_path = '/home/sarah/Documents/rosrepos/robis_ws/src/ROB-IS/hidden_costs_generator/maps/test_out.yaml'
generated_file_path_wtf = '/home/sarah/Documents/rosrepos/robis_ws/src/ROB-IS/hidden_costs_generator/maps/test_out_wtf.yaml'

with open(r'/home/sarah/Documents/rosrepos/robis_ws/src/ROB-IS/hidden_costs_generator/maps/objects-original.yaml') as test_object_file:
    map_file_info = yaml.load(test_object_file)
    print(map_file_info)
    mode = 'a' if os.path.exists(generated_file_path) else 'w'
    with open(generated_file_path, mode) as yaml_file:
        objects = yaml.dump(map_file_info, yaml_file)
    print(objects)
    yaml_file.close()

with open(r'/home/sarah/Documents/rosrepos/robis_ws/src/ROB-IS/hidden_costs_generator/maps/objects.yaml') as wtf_object_file:
    wtf_file_info = yaml.load(wtf_object_file)
    print(wtf_file_info)
    mode = 'a' if os.path.exists(generated_file_path_wtf) else 'w'
    with open(generated_file_path_wtf, mode) as yaml_file:
        objects = yaml.dump(wtf_file_info, yaml_file)
    print(objects)
    yaml_file.close()


