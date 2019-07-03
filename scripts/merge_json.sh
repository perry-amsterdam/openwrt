#!/bin/sh

JSON_DIR="$TOPDIR/json/"
JSON_FILE="$JSON_DIR/devices.json"
SCRIPTS_DIR="${SCRIPTS_DIR:-./scripts}"
rm -rf "$JSON_FILE"
DEVICES="$(ls $JSON_DIR)"

cat <<- EOF > "$JSON_FILE"
{
    "release_version": "$RELEASE_VERSION",
    "release_commit": "$($SCRIPTS_DIR/getver.sh)",
    "devices": {
    }
}
EOF

SEP=""

for DEVICE in $DEVICES ; do
    DEVICE_ID="$(basename $DEVICE .json)"
    head -n -2 $JSON_FILE > ${JSON_FILE}_tmp
    mv ${JSON_FILE}_tmp ${JSON_FILE}
    cat <<- EOF >> $JSON_FILE
        ${SEP}$(cat $JSON_DIR/$DEVICE | head -n 3 | tail -n 1 | sed 's/    "title": //' | sed 's/,//'): "$DEVICE_ID"
    }
}
EOF
    SEP=""","""
done
