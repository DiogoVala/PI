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
    x: 100
    y: 30
    width: 230
    height: 160
    clip: true
    ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
    ScrollBar.vertical.policy: ScrollBar.AlwaysOn

        Label {
            text: "Bem vindo ao manual.
Na página 'Parâmetros', pode definir e
visualizar os diversos valores atuais
dos parâmetros do sistema. Para isso,
basta escolher o parâmetro e o seu
valor atual aparecerá no ecrã. Para
escolher outro valor pode usar a barra
deslizante e, no fim, validar o valor.
Na página 'Gráficos', pode visualizar
dados em tempo real. De notar que existem
3 possíveis gráficos que pode visualizar.
Para ir para a página inicial, utilize 
o botão H.
Por fim, tem a página de mais
informações onde pode encontrar
diversas informações de como operar
este dispositivo e como contactar
os responsáveis pela manutenção."
            //horizontalAlignment: Text.AlignHCenter
            anchors.centerIn: parent
        }
    }
}

