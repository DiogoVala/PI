import QtQuick.Controls 2.3
import QtQuick 2.11
import QtCharts 2.15

ApplicationWindow {
    id: mainWindow
    title: "My Application"
    width: 400
    height: 400

    Item {
        id: ydataitem
        function ydatavalues() {
            var ydata = []
            for (var i = 0; i < 10; i++) {
                ydata.push(Math.random()*20)
                //console.debug(ydata)
            }
            return ydata
        }
        Component.onCompleted: ydatavalues()
    }

    Item {
        id: xdataitem
        function xdatavalues() {
            var xdata = []
            for (var i = 0; i < 10; i++) {
                xdata.push(i)
                //console.debug(xdata)
            }
            return xdata
        }
        Component.onCompleted: xdatavalues()
    }

    ChartView {
        id: chart
        title: "data"
        antialiasing: true // make the line data more smooth
        //animationOptions: ChartView.SeriesAnimations // animation of the data
        legend.visible:false
        margins.left: 10
        margins.right: 10
        margins.top: 10
        margins.bottom: 10
        property int timer: 0
        anchors.fill: parent

        property var ydataplot: ydataitem.ydatavalues()
        property var xdataplot: xdataitem.xdatavalues()

        ValueAxis {
            id: myAxisX
            min: 0
            max: 10
            tickCount: 11 // how many grid lines are drawn on the chart
        }

        ValueAxis {
            id: myAxisY
            min: 0
            max: 20
            tickCount: 5
        }
    
        LineSeries {
            id: lineSeries
            name: "LineSeries"
            axisX: myAxisX
            axisY: myAxisY
        }

        Item {
            id: plotitem

            function plotvalues() {

                chart.ydataplot.shift()
                chart.ydataplot[9] = Math.random()*20

                //console.debug(chart.ydataplot)

                for (var i = 0; i < 10; i++) {
                    lineSeries.append(chart.xdataplot[i],chart.ydataplot[i])
                    chart.timer = chart.timer + 1
                    console.debug(chart.xdataplot[i])
                    console.debug(chart.ydataplot[i])
                }
            } 
            Component.onCompleted: plotvalues()
        }

        Item {
            id: clearitem
            function clearvalues() {
                linwSeries.removeAllSeries
            } 
            Component.onCompleted: clearvalues()
        }
    
        Timer {
            id: time
            interval: 500 // in ms
            running: true
            repeat: true
            onTriggered: {
                plotitem.plotvalues()
                clearitem.clearvalues()
            }
        } 
    }
}