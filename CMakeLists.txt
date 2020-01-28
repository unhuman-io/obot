cmake_minimum_required(VERSION 3.0)
project(robot)

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "Lee Magnusson")
set(CPACK_PACKAGE_VERSION 0.1.0)
include(CPack)

install(FILES jetbot-camera-client.service DESTINATION /usr/lib/systemd/system) #/etc/systemd/user/)
install(PROGRAMS jetbot_camera_display.sh jetbot_camera_run.sh DESTINATION /usr/bin)