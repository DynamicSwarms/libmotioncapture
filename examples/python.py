import motioncapture
import argparse


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("type")
    parser.add_argument("hostname")
    parser.add_argument("full_pointcloud")
    
    args = parser.parse_args()

    mc = motioncapture.connect(args.type, {"hostname": args.hostname, "add_labeled_markers_to_pointcloud": args.full_pointcloud})

    while True:
        mc.waitForNextFrame()
        for name, obj in mc.rigidBodies.items():
            print(name, obj.position, obj.rotation.z)
        print("#Points in PointCloud", len(mc.pointCloud))
