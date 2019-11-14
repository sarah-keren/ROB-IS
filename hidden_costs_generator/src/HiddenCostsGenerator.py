#!/usr/bin/env python
import math  
import sys, os
from nav_msgs.msg import OccupancyGrid
import yaml
import cv2 
import random

# taken from https://stackabuse.com/reading-and-writing-yaml-to-a-file-in-python/

#Generate yaml files that contain information about the hidden costs 
class HiddenCostsGenerator:

    def __init__(self, map_yaml_file_path, generated_file_path):
        self.map_yaml_file_path = map_yaml_file_path
        self.generated_file_path = generated_file_path
        self.index = 0
        self.map_height = 0 
        self.map_width = 0  
        self.resolution = 0
        self.map_file_info = None
        self.map_image_path = None


    def initialize(self):

        with open(r'%s'%self.map_yaml_file_path) as map_yaml_file:
            self.map_file_info = yaml.full_load(map_yaml_file)

	    relative_map_image_path = self.map_file_info['image']
        map_dir = os.path.dirname(self.map_yaml_file_path)
        self.map_image_path = os.path.join(map_dir,relative_map_image_path) 	    
        
        self.map_img = cv2.imread(self.map_image_path) 
        [height, width, channels] = self.map_img.shape
        self.map_height = height
        self.map_width = width
        self.resolution = self.map_file_info['resolution']

    def generate_yaml_file(self, num_of_doughnuts, num_of_bananas, num_of_hppits=0):#, clusters_dimensions):
        
        # populate the objects_dict
        [doughnuts,bananas, hppits]= self.generate_objects(num_of_doughnuts, num_of_bananas, num_of_hppits)
     
        objects_dict = {}
        objects_dict['doughnuts'] = doughnuts
        objects_dict['bananas'] = bananas
        objects_dict['hppits'] = hppits

        # populate file  
        mode = 'w+' if os.path.exists(self.generated_file_path) else 'w'
        with open(self.generated_file_path, mode) as yaml_file:
            objects = yaml.dump(objects_dict,yaml_file)
            yaml_file.close()

        print('completed generate_yaml_file')

    def generate_objects(self, num_of_doughnuts, num_of_bananas, num_of_hppits):
        doughnuts = list()
        bananas = list()
        hppits = list()
        reference_point = [None, None]

        for i in xrange(0,num_of_doughnuts):
            print('i: %d'%i)    
            reference_point = self.generate_reference_point()
            doughnuts.append(self.generate_doughnut(reference_point))
         
        for i in xrange(0,num_of_bananas):
            print('i: %d'%i)  
            reference_point = self.generate_reference_point()  
            bananas.append(self.generate_banana(reference_point))

        for i in xrange(0,num_of_hppits):
            print('i: %d'%i)    
            reference_point = self.generate_reference_point()
            hppits.append(self.generate_hppit(reference_point))


        return [doughnuts,bananas,hppits]

    # p, c, angle, arclen, mu, sigma        
    def generate_banana(self, reference_point):
        self.index += 1
        p = reference_point[0]  #(self.index)%self.map_width#27.6 #
        c = reference_point[1]  #(self.index)%self.map_height#32.3#
        angle = 0.75
        arclen = 1.57
        mu = 0.5 
        sigma = 0.15
        print('generating banana')
        #return [p, c, angle, arclen, mu, sigma]
        object_dict= {'name': 'banana_%d'%self.index, 'x': p, 'y': c, 'radius': mu, 'angle': angle,  'arclen': arclen, 'std_dev': sigma, 'type': 'banana'}
        return object_dict

    # p, c, mu, sigma
    def generate_doughnut(self, reference_point):
        self.index += 1
        p = reference_point[0]  #4.17 +self.index*0.2 
        c = reference_point[1]  #round(38.8 +self.index*0.2,2)
        mu = 1.3 #0.5 
        sigma = 0.3 #0.75 
        print('generating doughnut')
        #return [p, c, mu, sigma]
        object_dict= {'name': 'dougnut_%d'%self.index,'x': p, 'y': c, 'radius': mu, 'std_dev': sigma, 'type': 'doughnut'}
        return object_dict
    
    # p, c, mu, sigma
    def generate_hppit(self, reference_point):
        self.index += 1
        p = reference_point[0]  #4.17 +self.index*0.2 
        c = reference_point[1]  #round(38.8 +self.index*0.2,2)
        mu = 1.3 #0.5 
        sigma = 0.3 #0.75 
        print('generating hppit')
        #return [p, c, mu, sigma]
        object_dict= {'name': 'hppit_%d'%self.index,'x': p, 'y': c, 'radius': mu, 'std_dev': sigma, 'type': 'doughnut'}
        return object_dict     

    def generate_reference_point(self, ranges=None):

        is_in_collision = True
        x_val_pixel = -1
        x_val = -1
        y_val_pixel = -1        
        y_val = -1
        while is_in_collision:

            #column   
            cell_x = random.randint(1,self.map_width-1)
            map_x = cell_x*self.resolution
            map_x = round(map_x,2)

            #row
            cell_y = random.randint(1,self.map_height-1)
            map_y = cell_y*self.resolution
            map_y = round(map_y,2)
            
            #check for collisions        
            point_RGB_value = self.map_img[cell_y,cell_x]
            print('point_RGB_value:')
            print(point_RGB_value)
            #point_prob_value = point_RGB_value[0]+point_RGB_value[1]+point_RGB_value[2]/255 #(R+G+B)/255
            val_avg = (int(point_RGB_value[0])+int(point_RGB_value[1])+int(point_RGB_value[2]))/3
            #occ = (255 - color_avg) / 255.0;
            point_prob_value = (255 - val_avg )/255.0 
        
            print('point_prob_value:')        
            print(point_prob_value)
            if point_prob_value < self.map_file_info['free_thresh']:
                is_in_collision = False
            else:
                print("found a collision in cell (%d,%d)- resampling"%(cell_y, cell_x))        

        return [map_x, map_y] 

if __name__ == '__main__':

    map_file_path = sys.argv[1]        
    generated_file_path = sys.argv[2]     
    num_dougnuts = int(sys.argv[3])
    num_bananas =  int(sys.argv[4])
    num_of_hppits = 0
    if len(sys.argv)>5	:	
        num_of_hppits =  int(sys.argv[5])

    generator = HiddenCostsGenerator(map_file_path, generated_file_path)
    generator.initialize()
    generator.generate_yaml_file(num_dougnuts, num_bananas, num_of_hppits)

