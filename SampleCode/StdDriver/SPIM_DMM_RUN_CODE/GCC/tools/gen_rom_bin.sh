#!/bin/sh

if [ $# -ne 2 ] ; then
    echo "Usage: $0 $1 $2"
    exit
fi

set -x

ProjName=$2
ProjRelDirPath=$1"/Release"

script_path="$(dirname "$0")"
hex_all=${ProjRelDirPath}"/"${ProjName}".hex"
hex_aprom=${ProjRelDirPath}"/"${ProjName}"_aprom.hex"
bin_aprom=${ProjRelDirPath}"/"${ProjName}"_aprom.bin"
hex_spim=${ProjRelDirPath}"/"${ProjName}"_spim.hex"
bin_spim=${ProjRelDirPath}"/"${ProjName}"_spim.bin"

"${script_path}/srec_cat.exe" "${hex_all}" -intel -crop 0x00000000 0x000FFFFF -o "${hex_aprom}" -intel -obs=16

"${script_path}/srec_cat.exe" "${hex_aprom}" -intel -offset -0x00000000 -o "${bin_aprom}" -binary

"${script_path}/srec_cat.exe" "${hex_all}" -intel -crop 0x00100000 0x020FFFFF -o "${hex_spim}" -intel -obs=16

"${script_path}/srec_cat.exe" "${hex_spim}" -intel -offset -0x00100000 -o "${bin_spim}" -binary

set +x