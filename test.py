import onnx

# Load the ONNX model
model = onnx.load("models/policy_V1.onnx")

# Print a human-readable representation of the model graph
#print(onnx.helper.printable_graph(model.graph))
