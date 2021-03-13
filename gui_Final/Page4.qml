import QtQuick.Controls 2.15
import QtQuick 2.11

Page {
    Column {
    spacing: 2
        Button {
            text: "Def. Param"
            width: 80
            height: 72

            onClicked: {
                mystackview.push(page2);
            }
        }
        Button {
            text: "Graf."
            width: 80
            height: 72

            onClicked: {
                mystackview.push(page3);
            }
        }
        Button {
            text: "Info."
            width: 80
            height: 72

            onClicked: {
                mystackview.push(page4);
            }
        }
    }
    Button {
            x: 365
            y: 200
            text: "H"
            width: 20
            height: 20
            onClicked: {
                mystackview.push(page1);
            }
    }
    Label {
        text: "Pagina Info"
        font.pixelSize : 20
        x: 160
        y: 90
    }
}

