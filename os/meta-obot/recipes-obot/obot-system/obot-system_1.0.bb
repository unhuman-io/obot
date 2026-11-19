SUMMARY = "System settings for realtime control on the obot image"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

inherit systemd

SRC_URI = " \
    file://obot-rt-tuning \
    file://obot-rt-tuning.service \
    file://90-obot-realtime.conf \
    file://obot-modules.conf \
"
S = "${UNPACKDIR}"

OBOT_RT_CPU ??= "-1"

do_install() {
    install -d ${D}${libexecdir}/obot ${D}${systemd_system_unitdir} ${D}${sysconfdir}/sysctl.d \
               ${D}${sysconfdir}/modules-load.d ${D}${sysconfdir}/default
    install -m 0755 ${S}/obot-rt-tuning ${D}${libexecdir}/obot/
    install -m 0644 ${S}/obot-rt-tuning.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${S}/90-obot-realtime.conf ${D}${sysconfdir}/sysctl.d/
    install -m 0644 ${S}/obot-modules.conf ${D}${sysconfdir}/modules-load.d/obot.conf
    echo "OBOT_RT_CPU=${OBOT_RT_CPU}" > ${D}${sysconfdir}/default/obot-rt-tuning
}

SYSTEMD_SERVICE:${PN} = "obot-rt-tuning.service"
SYSTEMD_AUTO_ENABLE = "enable"
FILES:${PN} += "${libexecdir}/obot ${systemd_system_unitdir}"
PACKAGE_ARCH = "${MACHINE_ARCH}"
