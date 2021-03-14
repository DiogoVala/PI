import QtQuick.Controls 2.3
import QtQuick 2.11
import QtCharts 2.15

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
    ScrollView {
    x: 140
    y: 30
    width: 230
    height: 160
    clip: true
    ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
    ScrollBar.vertical.policy: ScrollBar.AlwaysOn

        Label {
            text: "1º isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
isto é só um exemplo
16º isto é só um exemplo"
        }
    }
}

