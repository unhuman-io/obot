SUMMARY = "obot realtime actuator daemon"
DESCRIPTION = "obot-rtd runs the CAN control cycle on an isolated CPU and exposes the robot \
through shared memory. Built from the checkout this layer is part of."
HOMEPAGE = "https://github.com/unhuman-io/obot"
LICENSE = "Unlicense & MIT"
LIC_FILES_CHKSUM = " \
    file://${OBOT_SRC_DIR}/UNLICENSE.txt;md5=7246f848faa4e9c9fc0ea91122d6e680 \
    file://third_party/tomlplusplus/LICENSE;md5=90960f22c10049c117d56ed2ee5ee167 \
    file://third_party/motor-realtime/UNLICENSE.txt;md5=1d267ceb3a8d8f75f1be3011ee4cbf53 \
"

inherit cmake externalsrc systemd useradd

# motor-realtime (rt/third_party/motor-realtime, a git submodule: check out with
# --recurse-submodules) carries all communication with the actuators and finds USB boards
# through libudev.
DEPENDS += "udev"

EXTERNALSRC = "${OBOT_SRC_DIR}/rt"

# The tests run in CI on the build host; the target package carries only the programs.
EXTRA_OECMAKE = "-DOBOT_RT_BUILD_TESTS=OFF"

SRC_URI = " \
    file://obot-rtd.service \
    file://obot-can.service \
    file://obot-can-setup \
"

OBOT_RT_CPU ??= "-1"

do_install:append() {
    install -d ${D}${sysconfdir}/obot
    install -m 0644 ${OBOT_SRC_DIR}/config/obot3.toml ${D}${sysconfdir}/obot/robot.toml

    install -d ${D}${sysconfdir}/default
    echo "OBOT_RT_CPU=${OBOT_RT_CPU}" > ${D}${sysconfdir}/default/obot-rtd

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${UNPACKDIR}/obot-rtd.service ${UNPACKDIR}/obot-can.service ${D}${systemd_system_unitdir}/
    install -d ${D}${libexecdir}/obot
    install -m 0755 ${UNPACKDIR}/obot-can-setup ${D}${libexecdir}/obot/
}

USERADD_PACKAGES = "${PN}"
GROUPADD_PARAM:${PN} = "--system obot"
USERADD_PARAM:${PN} = "--system --no-create-home --gid obot --shell /sbin/nologin obot"

SYSTEMD_SERVICE:${PN} = "obot-rtd.service obot-can.service"
SYSTEMD_AUTO_ENABLE = "enable"

FILES:${PN} += "${systemd_system_unitdir} ${libexecdir}/obot"
CONFFILES:${PN} = "${sysconfdir}/obot/robot.toml ${sysconfdir}/default/obot-rtd"
RDEPENDS:${PN} += "iproute2"
# The command line arguments and the default file depend on the machine.
PACKAGE_ARCH = "${MACHINE_ARCH}"
