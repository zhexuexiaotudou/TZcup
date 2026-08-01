import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

Rectangle {
  anchors.fill: parent
  color: "#0d1724"
  implicitWidth: 360
  implicitHeight: 410

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 16
    spacing: 10

    Label {
      text: "TZcup 清扫任务"
      color: "#f4f7fb"
      font.pixelSize: 22
      font.bold: true
    }
    Label {
      text: SanitationMissionControl.sceneLabel
      color: "#9fb2c8"
      font.pixelSize: 14
    }

    Rectangle {
      Layout.fillWidth: true
      height: 58
      radius: 8
      color: "#16283b"
      border.color: "#2b4965"
      Column {
        anchors.centerIn: parent
        spacing: 3
        Label { text: "任务状态"; color: "#91a7bd"; font.pixelSize: 12 }
        Label {
          text: SanitationMissionControl.missionState
          color: "#57e0b5"
          font.pixelSize: 18
          font.bold: true
        }
      }
    }

    GridLayout {
      Layout.fillWidth: true
      columns: 2
      rowSpacing: 10
      columnSpacing: 10

      Button {
        Layout.fillWidth: true
        text: "▶  开始"
        highlighted: true
        onClicked: SanitationMissionControl.StartMission()
      }
      Button {
        Layout.fillWidth: true
        text: "Ⅱ  暂停"
        onClicked: SanitationMissionControl.PauseMission()
      }
      Button {
        Layout.fillWidth: true
        text: "↻  继续"
        onClicked: SanitationMissionControl.ResumeMission()
      }
      Button {
        Layout.fillWidth: true
        text: "■  停止任务"
        onClicked: SanitationMissionControl.StopMission()
      }
    }

    Label {
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      text: SanitationMissionControl.operatorMessage
      color: "#d9e4ef"
      font.pixelSize: 13
    }

    Item { Layout.fillHeight: true }

    Button {
      Layout.fillWidth: true
      text: "关闭 Gazebo"
      onClicked: SanitationMissionControl.CloseGazebo()
    }
    Label {
      Layout.fillWidth: true
      text: "任务暂停会关闭刷盘并取消当前 Nav2 goal；顶部世界暂停只冻结物理仿真。"
      wrapMode: Text.WordWrap
      color: "#7f93a9"
      font.pixelSize: 11
    }
  }
}
