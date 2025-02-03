#include "onnx.hpp"
OnnxHandler::OnnxHandler(const std::string _path, const int _num_inputs, const int _num_ouputs)
  : path(_path), num_inputs(_num_inputs), num_outputs(_num_ouputs) {

  init_onnx_session();

  // shape specifies the dimensions of the input/output sensor
  input_shape = { 1, num_inputs };
  output_shape = { 1, num_outputs };

  input_buffer = std::vector<float>(_num_inputs);
  output_buffer = std::vector<float>(_num_ouputs);

  auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

  input_tensor = Ort::Value::CreateTensor<float>(
    memory_info, input_buffer.data(), input_buffer.size(), input_shape.data(),
    input_shape.size());

  output_tensor = Ort::Value::CreateTensor<float>(
    memory_info, output_buffer.data(), output_buffer.size(), output_shape.data(),
    output_shape.size());

  std::cout << "Memory information : \n"
    << memory_info << std::endl;

  std::cout << "Input and output buffers have been initiliazed!\n"
    << "Address of the controller :\t" << memory_info << std::endl
    << "Input tensor " << input_tensor << "\t tensor size: " << input_buffer.size() << std::endl
    << "Output tensor " << output_tensor << "\t tensor size: " << output_buffer.size() << std::endl;
}

void OnnxHandler::init_onnx_session() {

  // onnx runtime sessions options
  Ort::SessionOptions options;
  OrtCUDAProviderOptions cuda_options;

  cuda_options.device_id = 0;
  cuda_options.gpu_mem_limit = SIZE_MAX;
  cuda_options.arena_extend_strategy = 0;
  cuda_options.do_copy_in_default_stream = 0;
  cuda_options.has_user_compute_stream = 0;
  cuda_options.user_compute_stream = nullptr;
  cuda_options.default_memory_arena_cfg = nullptr;
  options.AppendExecutionProvider_CUDA(cuda_options);

  // create session
  session = Ort::Session{ env, path.c_str(), options };
}

void OnnxHandler::run() {
    // onnx requires a pointer to an array of char
  const char* input_name[] = { INPUT_NAME };
  const char* output_name[] = { OUTPUT_NAME };

  int num_input_tensors = 1;
  int num_output_tensors = 1;

  session.Run(opt,
    input_name, &input_tensor, num_input_tensors,
    output_name, &output_tensor, num_output_tensors);
}
std::vector<float>& OnnxHandler::get_input_buffer() {
  return input_buffer;
}
std::vector<float>& OnnxHandler::get_output_buffer() {
  return output_buffer;
}