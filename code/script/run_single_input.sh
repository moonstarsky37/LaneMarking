# single_file=/data/220718_174955.las
single_file=/data/220718_174955.las
#dummy_folder/dummy_file.las

#road_type=highway
#road_type=urban
road_type=penghu

#gdb --args 
./bin/roadmarking ${single_file} ${single_file}_out ./model_pool/models_${road_type}_example/ ./config/parameter_${road_type}_example.txt
