import QtQuick.Controls 2.3
import QtQuick 2.11
import QtCharts 2.15

ApplicationWindow {
    id: mainWindow
    title: "My Application"
    width: 385
    height: 220
    
    StackView {
        id: mystackview
        anchors.fill: parent
        initialItem: page1
    }
    Component {
        id: page1
        Page1 {}
    }
    Component {
        id: page2
        Page2 {}
    }
    Component {
        id: page3
        Page3 {}
    }
    Component {
        id: page4
        Page4 {}
    }
    Component {
        id: page5
        Page5 {}
    }
    Component {
        id: page6
        Page6 {}
    }
    Component {
        id: page7
        Page7 {}
    }
}
