{
  "version": "2.0.0",
  "timestamp": "2026-11-09T14:44:00.438Z",
  "startPoint": {
    "x": 58.468023255813954,
    "y": 7.794331395348835,
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-musgmkpj-5lcn2w",
      "color": "#ffc516",
      "name": "Path 1",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 2000,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 58.41133720930233,
        "y": 16.51962209302326
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": 90,
        "endDeg": 90,
        "degrees": 0,
        "reverse": false,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      }
    },
    {
      "kind": "atomic",
      "id": "line-mw6x9mn6-ysptlb",
      "endPoint": {
        "x": 22.669266483797003,
        "y": 46.9313010657586,
        "locked": false
      },
      "controlPoints": [
        {
          "x": 57.89316860465116,
          "y": 35.47529069767443
        },
        {
          "x": 39.38299418604651,
          "y": 46.93638345796678
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": false,
        "degrees": 0,
        "startDeg": 0,
        "endDeg": 0,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      },
      "color": "#b07aa1",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 200,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "line-mw6xgezd-6snz0f",
      "endPoint": {
        "x": 8.664647683683246,
        "y": 46.96728336890183
      },
      "controlPoints": [],
      "heading": {
        "type": "constant",
        "reverse": true,
        "degrees": 180,
        "startDeg": 0,
        "endDeg": 0,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      },
      "color": "#e15759",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "line-mw9bsaoj-8ecl3r",
      "endPoint": {
        "x": 58,
        "y": 29
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "reverse": true,
        "degrees": 0,
        "startDeg": 180,
        "endDeg": 90,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      },
      "color": "#e15759",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 118.3,
          "y": 141.5
        },
        {
          "x": 135.5,
          "y": 118
        },
        {
          "x": 136.3,
          "y": 70.2
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.2,
          "y": 116.9
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-musgmkpj-5lcn2w"
    },
    {
      "kind": "path",
      "lineId": "line-mw6x9mn6-ysptlb"
    },
    {
      "kind": "path",
      "lineId": "line-mw6xgezd-6snz0f"
    },
    {
      "kind": "path",
      "lineId": "line-mw9bsaoj-8ecl3r"
    }
  ],
  "fieldPoints": [],
  "robot": {
    "forwardVelocity": 75,
    "strafeVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rLength": 16,
    "rWidth": 16,
    "safetyMargin": 1,
    "maxAcceleration": 30,
    "maxDeceleration": 30
  },
  "activePaths": []
}