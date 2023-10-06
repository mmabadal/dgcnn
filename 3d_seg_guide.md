
# installation

- Ubunut 20.04
- drivers: nvidia 525.105.17
- cuda: 10.1
- cudnn: 8.0

'''
 $ conda create --name XXXXX python=3.7
 $ conda activate XXXXX
 $ pip install tensorflow==1.14
 $ conda install natsort
 $ conda install scipy
 $ pip install open3d==0.9
 $ conda install matplotlib
 $ conda install -c conda-forge plyfile
 $ pip install scikit-image
 $ conda install -c anaconda protobuf
 $ pip install rospkg
 $ pip install rosnumpy
 $ pip install pyyaml
'''
 
# get data

- rosrun pcl_ros pointcloud_to_pcd input:=/stereo_narrow/points2

- pcd_to_ply.py

- marcar gt en ply 

- ply_to_txt.py

- txt_to_npy.py

- npy_to_h5.py


# data management

data
  classes.txt
  train_val
    train
      h5
    val
      h5
  test
    test1
      npy
    ...
  

# train and infer

- python3 train.py --path_data Desktop/data/train_val/ --cls 5 --log_dir RUNS/run_x --batch_size X  # con 32 no va

- python inference.py --path_data /home/data/txt/ --path_out /home/test/out --path_cls RUNS/4/cls.txt --model_path RUNS/4/ --test_name test1 --points_sub 128 --targets_path ../valve_targets/


# ROS    
- ROS melodic
- rosbag_play.launch
- decimate_x2.launch
- stereo_proc.launch
- Tools/launches/pc_voxel.launch
- pip install pyyaml
- conda install -c conda-forge rospkg

# Python2  NECESARIO PARA COGER TRANSFORMADAS ROS

pip2 install plyfile (numpy==1.11)

pip2 install tensorflow==1.14

pip2 install open3d==0.9 pyrsistent==0.16.1

pip2 install scipy

pip2 install scikit-image


