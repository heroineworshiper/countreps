# from https://github.com/NVIDIA/TensorRT/issues/1677

import onnx
import onnx_graphsurgeon as gs
import numpy as np

print("loading model")
#graph = gs.import_onnx(onnx.load("../openpose/models/pose/body_25/body25.onnx"))
#graph = gs.import_onnx(onnx.load("body25_fixed.onnx"))
#graph = gs.import_onnx(onnx.load("../trt_pose/tasks/human_pose/resnet18.onnx"))
graph = gs.import_onnx(onnx.load("body25_256x144_fixed.onnx"))



for node in graph.nodes:
    print("name=%s op=%s\n\tinputs=%s\n\toutputs=%s" % (node.name, node.op, str(node.inputs), str(node.outputs)))
    #print("name=%s %s" % (node.name, str(node.inputs[0].shape)))





