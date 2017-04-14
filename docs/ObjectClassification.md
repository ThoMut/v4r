# Object Classification by ESF

In this tutorial you will learn how to classify objects using the ESF descriptor. In our example, we will render views from mesh files and then classify point clouds by classifying extracted segments. We use the Cat10 models from 3dNet for training and the test clouds of 3dNet for testing.


## Rendering training views from mesh files
In the first step, we will render training views ( point clouds `*.pcd` ) from mesh files ( `*.ply`). The example program `example_depth_map_renderer` produces a set of training views (in the output folder given by argument `-o`) from a mesh input file (provided by argument `-i`). The training views are rendered by placing virtual cameras on a 3D sphere with radius `-r` around the mesh file; sampled on an icosahedron with subdivision `-s`. Camera parameters are provided by parameters `--fx, --fy`(focal length in x and y direction), `--cx, --cy`(central point of projection in x and y direction), and `--width, --height`(image width and height). To visualize, add argument `-v`, for help `-h`.

Note that the program above renders single `.ply` files only. To render a bunch of `.ply` files, there is a tool called `tool_create_classification_db_from_ply_files` in the V4R library. 

If you have not obtained data yet, you can download them using the script file. Go to the v4r root directory and run
```
./scripts/get_3dNet_Cat10.sh
```
This will download and extract the Cat10 models (42MB).




```
object-model-database  
│
└───class-name-1
        |
        └─── instance-name-1
│            │
│            └─── views
│                 │   cloud_000000.pcd
│                 │    cloud_00000x.pcd
│                 │   ...
│                 │   object_indices_000000.txt
│                 │   object_indices_00000x.txt
│                 │   ...
│                 │   pose_000000.txt
│                 │   pose_00000x.txt
│                 │   ...
│          │
│          └─── [trained-pipeline-1]*
│                 │   ...
│                 │
│          └─── [trained-pipeline-x]*
│                 │   ...
        └─── instance-name-x
│            │
│            └─── views
│                 │   cloud_000000.pcd
│                 │    cloud_00000x.pcd
│                 │   ...
│                 │   object_indices_000000.txt
│                 │   object_indices_00000x.txt
│                 │   ...
│                 │   pose_000000.txt
│                 │   pose_00000x.txt
│                 │   ...
│          │
│          └─── [trained-pipeline-1]*
│                 │   ...
│                 │
│          └─── [trained-pipeline-x]*
│                 │   ...
│   
│
└───class-name-2
        |
        └─── instance-name-1
│            │
│            └─── views
│                 │   cloud_000000.pcd
│                 │    cloud_00000x.pcd
│                 │   ...
│                 │   object_indices_000000.txt
│                 │   object_indices_00000x.txt
│                 │   ...
│                 │   pose_000000.txt
│                 │   pose_00000x.txt
│                 │   ...
│          │
│          └─── [trained-pipeline-1]*
│                 │   ...
│                 │
│          └─── [trained-pipeline-x]*
│                 │   ...
        └─── instance-name-x
│            │
│            └─── views
│                 │   cloud_000000.pcd
│                 │    cloud_00000x.pcd
│                 │   ...
│                 │   object_indices_000000.txt
│                 │   object_indices_00000x.txt
│                 │   ...
│                 │   pose_000000.txt
│                 │   pose_00000x.txt
│                 │   ...
│          │
│          └─── [trained-pipeline-1]*
│                 │   ...
│                 │
│          └─── [trained-pipeline-x]*
│                 │   ...
│ ...
```
`* this data / folder will be generated`


## Training a classifier  


If you also want some annotated example point clouds, you can obtain the test set from 3dNet (3.6GB) by running

```
./scripts/get_3dNet_test_data.sh
```

The files will be extracted in the `data/3dNet` directory.

## Usage
Assuming you built the examples samples, you can now run the classifier. If you run it for the first time, it will automatically render views by placing a virtual camera on an artificial sphere around the mesh models in `-i` and store them in the directory given by the `-m` argument. These views are then used for training the classifier, in our case by extracting ESF descriptors. For testing, it will segment the point cloud given by the argument `-t` by your method of choice (default: searching for a dominant plane and running Euclidean clustering for the points above). Each segment is then described by ESF and matched by nearest neighbor search to one of your learned object classes. The results will be stored in a text file which has the same name as the input cloud, just replacing the suffix from `.pcd` to `.anno_test` in the output directory specified by `-o`.  
```
./build/bin/example-esf_object_classifier -i data/3dNet/Cat10_ModelDatabase -m data/3dNet/Cat10_Views -t data/3dNet/Cat10_TestDatabase/pcd_binary/ -o /tmp/3dNet_ESF_results
```

## References
* https://repo.acin.tuwien.ac.at/tmp/permanent/3d-net.org/

*  Walter Wohlkinger, Aitor Aldoma Buchaca, Radu Rusu, Markus Vincze. "3DNet: Large-Scale Object Class Recognition from CAD Models". In IEEE International Conference on Robotics and Automation (ICRA), 2012.
