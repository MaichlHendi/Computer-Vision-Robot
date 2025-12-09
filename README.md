# Computer Vision Robot
<h2>Project overview:</h2>
This repository contains the complete workflow for developing, training, and deploying an object detection model using MediaPipe and TensorFlow Lite, optimized for execution on the Coral EdgeTPU. The project demonstrates the full pipeline from model training to exporting a quantized TFLite model, generating metadata for embedded applications, and integrating the model into a Python-based state machine system.

<h2>Project goals</h2>

  1. Line follow with the help of sensors
  2. Detect pressure plate and drive to it
  3. Return to line and enter maze
  4. Find a basket and pick it up
  5. Find exit and drive out

<h2>Repository Structure</h2>

<h3>mediapipe-object-detection-learning.ipynb</h3>
A Jupyter notebook that documents the entire model development process. It includes dataset preparation, model training using MediaPipe’s object detection framework, evaluation, model export, and quantization steps. The notebook also covers generating model metadata for use in embedded applications and preparing the final model for EdgeTPU compilation.

<h3>model_int8-all_edgetpu.tflite</h3>
The final TensorFlow Lite model used for inference. It is fully INT8-quantized and compiled for the Coral EdgeTPU. This model is designed for real-time, low-latency operation on embedded devices.

<h3>metadata-all.hpp</h3>
An automatically generated C++ header file containing metadata used to describe the model. This includes information such as input and output tensor specifications and label mappings. The file is intended for integration into C++ inference pipelines on resource-constrained systems.

<h3>statemachine.py</h3>
A Python implementation of a state machine that organizes the runtime behavior of the application. It manages application states, transitions, and event handling based on model inference results. This demonstrates the integration of the trained model into an operational software system.

<h3>README.md</h3>
Documentation describing the purpose, structure, and workflow of the project.


<h3>Demonstration</h3>
A video of the robot in action. 
