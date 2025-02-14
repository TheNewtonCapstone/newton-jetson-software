#pragma once

#include <iostream>
#include <cmath>
#include <cstdint>
#include <onnxruntime_c_api.h>
#include <onnxruntime_cxx_api.h>

class OnnxHandler
{
public:
  OnnxHandler(const std::string, const int, const int);
  void run();
  void init_onnx_session();
  std::vector<float>& get_input_buffer();
  std::vector<float>& get_output_buffer();

private:
  const char* INPUT_NAME = "observations";
  const char* OUTPUT_NAME = "actions";

  std::string path;
  int num_inputs;
  int num_outputs;

  Ort::Env env;
  Ort::Session session{ nullptr };
  Ort::RunOptions opt{ nullptr };

  Ort::Value input_tensor{ nullptr };
  // shape specifies the dimensions of the input/output sensor
  std::array<int64_t, 2> input_shape;

  std::vector<float> input_buffer;
  Ort::Value output_tensor{ nullptr };
  std::array<int64_t, 2> output_shape;
  std::vector<float> output_buffer;
};