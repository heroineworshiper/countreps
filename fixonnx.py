# from https://github.com/NVIDIA/TensorRT/issues/1677
# time python3 fixonnx.py

import onnx
import onnx_graphsurgeon as gs
import numpy as np
import sys

#IN_MODEL = "body25_128x224.onnx"
#OUT_MODEL = "body25_128x224_fixed.onnx"

IN_MODEL = sys.argv[1]
OUT_MODEL = sys.argv[2]

print("loading model %s" % IN_MODEL)
graph = gs.import_onnx(onnx.load(IN_MODEL))


# make input resizable
#tensors = graph.tensors()
#tensors["input"].shape[0] = gs.Tensor.DYNAMIC


for node in graph.nodes:
#     print("name=%s op=%sn\n\tinputs=%s\n\toutputs=%s" % (node.name, node.op, str(node.inputs), str(node.outputs)))
     if node.op == "PRelu":
         # Make the slope tensor broadcastable
         print("Fixing")
         slope_tensor = node.inputs[1]
         slope_tensor.values = np.expand_dims(slope_tensor.values, axis=(0, 2, 3))
print("saving model %s" % OUT_MODEL)

onnx.save(gs.export_onnx(graph), OUT_MODEL)





