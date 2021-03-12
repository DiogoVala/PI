import QtQuick.Controls 2.12

ApplicationWindow {
    title: "My Application"

    Button {
        text: "Push me"
    }

    ComboBox {
        model: ["First", "Second", "Third"]
    }

    Tumbler {
        model: 5
    }
}
