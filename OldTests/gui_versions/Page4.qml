import QtQuick.Controls 2.3
import QtQuick 2.11
import QtCharts 2.15

Page {
    Column {
    spacing: 2
        Button {
            text: "Contactos"
            width: 80
            height: 72

            onClicked: {
                mystackview.push(page4);
            }
        }
        Button {
            text: "Ajuda?"
            width: 80
            height: 72

            onClicked: {
                mystackview.push(page7);
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
    ScrollView {
    x: 140
    y: 30
    width: 230
    height: 160
    clip: true
    ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
    ScrollBar.vertical.policy: ScrollBar.AlwaysOn

        Label {
            text: "email:
geral@azevedosindustria.com

tel. +351 22 747 15 70
fax. +351 22 747 15 79"
        }
    }
}

