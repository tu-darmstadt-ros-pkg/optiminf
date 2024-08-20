#  Optiminf
## General
This package is designed for implementing and efficiently solving non-linear programs (NLP) with constraints.

The framework uses [Eigen](https://eigen.tuxfamily.org/) for representing vectors and matrices and the Open Source interior point solver [IPOPT](https://github.com/coin-or/Ipopt).
It can be combined with the capabilities of our [autojac](https://github.com/tu-darmstadt-ros-pkg/autojac) package to enable automatic differentiation and code generation for calculating the jacobian matrices utilizing [CppAD](https://github.com/coin-or/CppAD) and [CppADCodeGen](https://github.com/joaoleal/CppADCodeGen). 
We designed the framework such that it is well suited for repeatedly solving NLPs of the same or a similar structure with little overhead for e.g. memory allocation.
This has proven especially useful for model predictive control applications.

## Features
- convenient problem formulation using [Eigen](https://eigen.tuxfamily.org/)
- integration of the Open Source solver [IPOPT](https://github.com/coin-or/Ipopt)
- generic solver interface for integrating more solvers
- compatible with our [autojac](https://github.com/tu-darmstadt-ros-pkg/autojac) package for automatic differentiation
- integration of our parameter interface [paraminf](https://github.com/tu-darmstadt-ros-pkg/paraminf) for simple YAML-file based configuration avoiding the need for recompilation
- the modular design enables to easily activate and deactive costs & constraints 
- implementation of cubic Hermite splines
- designed for efficiency
  - utilization of sparse matrices
  - calculations used by multiple costs or constraints can be cached
  - code generation for autodiff can be used through our [autojac](https://github.com/tu-darmstadt-ros-pkg/autojac) package
  - API allows performing costly operations like memory allocations upfront, minimizing the overhead when solving similar problems in sequence
  

## Installation
While this package and the dependencies set up to be build using [ament](https://design.ros2.org/articles/ament.html), they have no ROS dependencies.
The package has been developed originally using catkin as build system. You can find this version on the "catkin_version" branch.
If you want to use the catkin version make sure to also switch to the "catkin_version" branches of paraminf and autojac.

1. Install IPOPT
```
sudo apt-get install coinor-libipopt-dev
```
2. Clone the following additional packages provided by tu-darmstadt-ros-pkg into your ROS workspace and perform the install steps described in their README files:
- [paraminf](https://github.com/tu-darmstadt-ros-pkg/paraminf)
- [autojac](https://github.com/tu-darmstadt-ros-pkg/autojac)
3. Build your workspace using `colcon build`.

## Documentation
When building the package you can use the flag '-DBUILD_DOC=TRUE' to build the documentation. You can access it in the doc folder afterwards.
The mainpage of the documentation also contains further details on how to implement and solve your own NLP.

## Example
The folders example and test contains the source code and the configuration for a demo application that can be executed using:
```
rosrun optiminf run-demo
```

## Future Development & Contribution
The project during which the package was developed has been discontinued.
But in case you find bugs, typos or have suggestions for improvements feel free to open an issue.
We would especially appreciate Pull Requests fixing open issues.

## Authors
- Felix Biemüller
- Alexander Gutte