variable "REGISTRY" {
  default = "unified_autonomy"
}

group "default" {
  targets = ["ros1_base", "ros2_base", "ros1_gbplanner", "ros2_sim", "cuda_pytorch", "ros2_cuda", "ros2_nmpc", "ros2_cbf", "ros1-bridge-builder", "ros2_ros1_bridge", "ros2_rl", "ros2_vlm"]
}

target "default" {
  network = "host"
}

// Base images (no dependencies)
target "ros1_base" {
  context    = "."
  dockerfile = "Dockerfile.ros1_base"
  tags       = ["${REGISTRY}:ros1_base"]
  network = "host"
}

target "ros2_base" {
  context    = "."
  dockerfile = "Dockerfile.ros2_base"
  tags       = ["${REGISTRY}:ros2_base"]
  network = "host"
}

target "ros1-bridge-builder" {
  context = "."
  dockerfile = "Dockerfile.ros1_bridge_builder"
  tags       = ["${REGISTRY}:ros1-bridge-builder"]
  network = "host"
}

target "ros2_ros1_bridge" {
  context    = "."
  dockerfile = "Dockerfile.ros2_ros1_bridge"
  tags       = ["${REGISTRY}:ros2_ros1_bridge"]
  contexts   = {
    "unified_autonomy:ros2_base" = "target:ros2_base"
  }
  network = "host"
}


// Derived images (depend on base images)
target "ros1_gbplanner" {
  context    = "."
  dockerfile = "Dockerfile.ros1_gbplanner"
  tags       = ["${REGISTRY}:ros1_gbplanner"]
  contexts   = {
    "unified_autonomy:ros1_base" = "target:ros1_base"
  }
  network = "host"
}

target "ros2_sim" {
  context    = "."
  dockerfile = "Dockerfile.ros2_sim"
  tags       = ["${REGISTRY}:ros2_sim"]
  contexts   = {
    "unified_autonomy:ros2_base" = "target:ros2_base"
  }
  network = "host"
}

target "cuda_pytorch" {
  context    = "."
  dockerfile = "Dockerfile.cuda_pytorch"
  tags       = ["${REGISTRY}:cuda_pytorch"]
  network = "host"
}

target "ros2_cuda" {
  context    = "."
  dockerfile = "Dockerfile.ros2_cuda"
  tags       = ["${REGISTRY}:ros2_cuda"]
  network = "host"
  contexts   = {
    "unified_autonomy:cuda_pytorch" = "target:cuda_pytorch"
  }
}

target "ros2_nmpc" {
  context    = "."
  dockerfile = "Dockerfile.ros2_nmpc"
  tags       = ["${REGISTRY}:ros2_nmpc"]
  network = "host"
  contexts   = {
    "unified_autonomy:ros2_cuda" = "target:ros2_cuda"
  }
}

target "ros2_rl" {
  context    = "."
  dockerfile = "Dockerfile.ros2_rl"
  tags       = ["${REGISTRY}:ros2_rl"]
  network = "host"
  contexts   = {
    "unified_autonomy:ros2_cuda" = "target:ros2_cuda"
  }
}

target "ros2_cbf" {
  context    = "."
  dockerfile = "Dockerfile.ros2_cbf"
  tags       = ["${REGISTRY}:ros2_cbf"]
  contexts   = {
    "unified_autonomy:ros2_base" = "target:ros2_base"
  }
  network = "host"
}

target "ros2_vlm" {
  context    = "."
  dockerfile = "Dockerfile.ros2_vlm"
  tags       = ["${REGISTRY}:ros2_vlm"]
  network = "host"
  contexts   = {
    "unified_autonomy:ros2_base" = "target:ros2_base"
  }
}