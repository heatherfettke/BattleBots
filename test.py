from ultralytics import YOLO
 
# Load the model.
model = YOLO('yolov8n.pt')
# Training.
results = model.detect(
   data='data.yaml',
   imgsz=640,
   epochs=10,
   batch=8,
   name='yolov8n_custom')
