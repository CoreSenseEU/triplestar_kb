cmake_minimum_required(VERSION 3.10)
project(triplestar_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY bringup_template DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # the bringup template package.xml holds cookiecutter placeholders that cannot
  # pass schema validation, so xmllint checks every other XML file explicitly
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_xmllint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint(package.xml "bringup_template/{{cookiecutter.bringup_name}}/launch")
endif()

ament_package()
