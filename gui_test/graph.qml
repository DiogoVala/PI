import QtQuick.Controls 2.3
import QtQuick 2.11
import QtCharts 2.15

ApplicationWindow {
    id: mainWindow
    title: "My Application"
    width: 400
    height: 400

    ChartView {
        id: chart
        title: "data"
        antialiasing: true
        animationOptions: ChartView.SeriesAnimations
        legend.visible:false
        margins.left: 10
        margins.right: 10
        margins.top: 10
        margins.bottom: 10
        property int timer: 0
        anchors.fill: parent

        ValueAxis {
            id: myAxisX
            min: 0
            max: 10>chart.timer? 10:chart.timer+1
            tickCount: 11
        }

        ValueAxis {
            id: myAxisY
            min: 0
            max: 50
            tickCount: 6
        }
    
        LineSeries {
            id: lineSeries
            name: "LineSeries"
            axisX: myAxisX
            axisY: myAxisY
        }
    
        Timer {
            id: time
            interval: 1000
            running: true
            repeat: true
            onTriggered: {
                lineSeries.append(chart.timer,Math.random()*50)
                chart.timer = chart.timer + 1
            }
        }
    }
}