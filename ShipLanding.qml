import QtQuick 2.2

import QGroundControl.Controls      1.0
import QGroundControl.Palette       1.0
import QGroundControl.ScreenTools   1.0
import QGroundControl.Controllers   1.0

Rectangle {
    anchors.fill:   parent
    color:          qgcPal.window

    CustomCommandWidgetController {
        id:         controller
        factPanel:  panel
    }

    QGCPalette { id: qgcPal; colorGroupEnabled: enabled }

    Column {
        spacing: ScreenTools.defaultFontPixelHeight

        QGCButton {
            text: "Start Shiplanding Procedure"
            onClicked: controller.start_landing()
        }
    }
}
