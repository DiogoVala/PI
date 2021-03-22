import QtQuick.Controls 2.3
import QtQuick 2.11
import QtCharts 2.15
import QtQuick.Layouts 1.3
import QtQuick.Controls.Material 2.0

Page {
    Column {
    spacing: 2
        Button {
            text: "Parâmetros"
            width: 80
            height: 72
            font.pixelSize : 12

            onClicked: {
                mystackview.push(page2);
            }
        }
        Button {
            text: "Gráficos"
            width: 80
            height: 72
            font.pixelSize : 12

            onClicked: {
                mystackview.push(page3);
            }
        }
        Button {
            text: "Informações"
            width: 80
            height: 72
            font.pixelSize : 12

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
    Tumbler {
        id: values
        model: 100
        x: 300
        y: 20
        width: 20
        height: 100
    }
    property real temperature;
    property real prefusion;
    property real voltage;
    property real current;

    Component.onCompleted: {
         temperature = 0;
    }

    ComboBox {
        x: 100
        y: 50
        id: varibles
        model: ListModel {
            id: model;
            ListElement{
                name : "Def. Temp."
            }
            ListElement{
                name : "Temp. Pré Fusão"
            }
            ListElement{
                name : "Calib. Tensão"
            }
            ListElement{
                name : "Calib. Corrente"
            }
        }
        onCurrentIndexChanged: {
            if ("0" == varibles.currentIndex){
                scaletext.text = temperature
            }
            if ("1" == varibles.currentIndex){
                scaletext.text = prefusion
            }
            if ("2" == varibles.currentIndex){
                scaletext.text = voltage
            }
            if ("3" == varibles.currentIndex){
                scaletext.text = current
            }
        }
    }


    TextField {
        id: scaletext
        readOnly: true
        x: 100
        y: 100
        width: 140
        height: 30
        text: varibles.currentIndex
    }

    Button {
        x: 260
        y: 140
        text: "Validar"
        onClicked: {
            scaletext.text = values.currentIndex
            if ("Set Temperature" == varibles.currentText){
                temperature = values.currentIndex
            }
            if ("Pre-Fusion Temperature" == varibles.currentText){
                prefusion = values.currentIndex
            }
            if ("Voltage Calibration" == varibles.currentText){
                voltage = values.currentIndex
            }
            if ("Current Calibration" == varibles.currentText){
                current = values.currentIndex
            }
        }
    }

}

