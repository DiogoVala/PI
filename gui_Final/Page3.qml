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

        //anchors.fill: parent
        x: 80
        y: 0
        height: 200
        width: 300

        property var ydataplot: ydataitem.ydatavalues()
        property var xdataplot: xdataitem.xdatavalues()

        ValueAxis {
            id: myAxisX1
            min: 0
            max: 10
            tickCount: 11 // how many grid lines are drawn on the chart
            labelsFont:Qt.font({pointSize: 6})
        }

        ValueAxis {
            id: myAxisY1
            min: 0
            max: 20
            tickCount: 5
            labelsFont:Qt.font({pointSize: 6})
        }

        LineSeries {
            id: lineSeries1
            name: "LineSeries1"
            axisX: myAxisX1
            axisY: myAxisY1
        }

        Item {
            id: plotitem

            function plotvalues() {
                var myAxisX = chart.axisX(lineSeries1)
                var myAxisY = chart.axisY(lineSeries1)
                var series = chart.createSeries(LineSeries, "LineSeries", myAxisX, myAxisY)

                chart.ydataplot.shift()
                chart.ydataplot[9] = Math.random()*20

                var xdatasave = chart.xdataplot
                var ydatasave = chart.ydataplot

                //console.debug(ydatasave)

                //console.debug(chart.ydataplot)

                for (var i = 0; i < 10; i++) {
                    series.append(xdatasave[i],ydatasave[i])
                    //console.debug(chart.xdataplot[i])
                    //console.debug(chart.ydataplot[i])
                }
                return series
            }
            Component.onCompleted: plotvalues()
        }

        Item {
            id: clearitem
            function clearvalues() {
                //console.debug(plotitem.plotvalues())
                chart.removeAllSeries(plotitem.plotvalues())
            }
            Component.onCompleted: clearvalues()
        }

        Timer {
            id: time
            interval: 500 // in ms
            running: true
            repeat: true
            onTriggered: {
                clearitem.clearvalues()
                plotitem.plotvalues()
            }
        }
    }
}

