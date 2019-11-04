#!/usr/bin/env python
import rospy
import math  
import sys, os
from nav_msgs.msg import OccupancyGrid
import yaml

# taken from https://stackabuse.com/reading-and-writing-yaml-to-a-file-in-python/

#Generate yaml files that contain information about the hidden costs 
class HiddenCostsGenerator:

    def __init__(self, map_file_path, generated_file_path):
        self.map_file_path = map_file_path
        self.generated_file_path = generated_file_path
        rospy.loginfo("HiddenCostsGenerator initialized")
        self.index = 0

    def initialize(self):
        with open(r'%s'%self.map_file_path) as map_file:
    	     map_file_info = yaml.full_load(map_file)
             print('map_file_info:')	     
             print(map_file_info)	     

    def generate_yaml_file(self, num_of_doughnuts, num_of_bananas):
        
        # populate the objects_dict
        [doughnuts,bananas]= self.generate_objects(num_of_doughnuts, num_of_bananas)	
     
        print('doughnuts:')	
        print(doughnuts)

        print('bananas:')	
        print(bananas)
	
	objects_dict = {}
        objects_dict['doughnuts'] = doughnuts
        objects_dict['bananas'] = bananas

	print(objects_dict)

	# populate file  
	mode = 'a' if os.path.exists(self.generated_file_path) else 'w'
	with open(self.generated_file_path, mode) as yaml_file:
            objects = yaml.dump(objects_dict,yaml_file)
	    print(objects)
        yaml_file.close()

	print('completed generate_yaml_file')

    def generate_objects(self, num_of_doughnuts, num_of_bananas):
        doughnuts = list()
	bananas = list()

	for i in xrange(0,num_of_doughnuts):
	    print('i: %d'%i)    
	    doughnuts.append(self.generate_doughnut())
         
	for i in xrange(0,num_of_bananas):
	    print('i: %d'%i)    
	    bananas.append(self.generate_banana())

	return [doughnuts,bananas]

    # p, c, angle, arclen, mu, sigma	
    def generate_banana(self):

        self.index += 1
        p = 27.6
        c = 32.3
        angle = 0.75
        arclen = 1.57
        mu = 0.5 
        sigma = 0.15
	print('generating banana')
        #return ['Banana%d'%self.index, {'p': p, 'c': c, 'angle':angle, 'arclen':arclen, 'mu':mu, 'sigma': sigma}]
        #return [p, c, angle, arclen, mu, sigma]
        return [p, c, angle, arclen, mu, sigma]

    # p, c, mu, sigma
    def generate_doughnut(self):
        self.index += 1
	p = 4.17 +self.index*0.2
        c = round(38.8 +self.index*0.2,2)
        mu = 0.5 
        sigma = 0.75 
        print('generating doughnut')
        return [p, c, mu, sigma]


	
#    def generate_yaml_file(self, num_of_doughnuts, num_of_bananas):
#	mode = 'a' if os.path.exists(self.generated_file_path) else 'w'
#	object_dict= {'name': 'Silenthand Olleander', 'race': 'Human','traits': ['ONE_HAND', 'ONE_EYE']}
#       with open(self.generated_file_path, mode) as yaml_file:
#            objects = yaml.dump(dict_,yaml_file)
#	    print(objects)
#        yaml_file.close()
#	print('completed generate_yaml_file')


if __name__ == '__main__':

    rospy.init_node('hidden_costs_generator')
    map_file_path = sys.argv[1]	
    generated_file_path = sys.argv[2]	
    try:
        generator = HiddenCostsGenerator(map_file_path, generated_file_path)
	generator.initialize()
        generator.generate_yaml_file(3, 2)

    except rospy.ROSInterruptException:
        pass
