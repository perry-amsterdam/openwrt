#!/bin/sh

IMAGE_HASH=$(sha256sum "$BIN_DIR/$IMAGE_NAME" | awk '{printf($1)}')
JSON_FILE="$TOPDIR/json/$DEVICE_ID.json"
SEP=""",
        """

mkdir -p "$TOPDIR/json/"

if [ ! -f "$JSON_FILE" ]; then
    SEP=""
    # create new json file file
    cat <<- EOF > $JSON_FILE
{
    "id": "$DEVICE_ID",
    "title": "$DEVICE_TITLE",
    "vendor": "$DEVICE_VENDOR",
    "model": "$DEVICE_MODEL",
    "variant": "$DEVICE_VARIANT",
    "supported_devices": [ $(echo $SUPPORTED_DEVICES | sed -e 's/^/"/' -e 's/$/"/' -e 's/ /", "/') ],
    "target": "$TARGET",
    "subtarget": "$SUBTARGET",
    "images": [
    ]
}
EOF
fi

if ! grep -q "$IMAGE_HASH" "$JSON_FILE"; then
    # append new image entry
    head -n -2 $JSON_FILE > ${JSON_FILE}_tmp
    mv ${JSON_FILE}_tmp ${JSON_FILE}
    cat <<- EOF >> $JSON_FILE
    ${SEP}{
        "type": "$IMAGE_TYPE", "name": "$IMAGE_NAME", "hash": "$IMAGE_HASH" }
    ]
}
EOF
fi
