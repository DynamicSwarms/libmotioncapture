import motioncapture
import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("type")
    parser.add_argument("hostname")
    parser.add_argument("--add-labeled-markers", action="store_true",
                        help="Include labeled Vicon markers in the point cloud")
    args = parser.parse_args()

    mc = motioncapture.connect(args.type, {"hostname": args.hostname,
                                           "add_labeled_markers_to_pointcloud": str(args.add_labeled_markers).lower()})
    while True:
        mc.waitForNextFrame()
        for name, obj in mc.rigidBodies.items():
            print(name, obj.position, obj.rotation.z)
