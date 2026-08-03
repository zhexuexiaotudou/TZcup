import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

Rectangle {
  id: root
  anchors.fill: parent
  color: "#0b1522"
  implicitWidth: 360
  implicitHeight: 820
  property var telemetry: ({"state": "WAITING_FOR_DATA"})
  property bool showPlanned: true
  property bool showActual: true
  property bool showRepairs: true

  function updateTelemetry() {
    try {
      telemetry = JSON.parse(SanitationMissionControl.telemetryJson)
      mapCanvas.requestPaint()
    } catch (error) {
      telemetry = {"state": "INVALID_TELEMETRY"}
    }
  }
  function value(name, fallback) {
    return telemetry[name] === undefined ? fallback : telemetry[name]
  }
  function number(name, digits, suffix) {
    var v = Number(value(name, 0))
    return (isFinite(v) ? v.toFixed(digits) : "--") + suffix
  }

  Connections {
    target: SanitationMissionControl
    function onTelemetryJsonChanged() { root.updateTelemetry() }
  }
  Component.onCompleted: updateTelemetry()

  ScrollView {
    anchors.fill: parent
    clip: true
    contentWidth: availableWidth

    ColumnLayout {
      width: parent.width
      spacing: 10

      Item { Layout.preferredHeight: 4 }
      Label {
        Layout.leftMargin: 16
        text: "TZcup 清扫任务"
        color: "#f4f7fb"
        font.pixelSize: 22
        font.bold: true
      }
      Label {
        Layout.leftMargin: 16
        text: SanitationMissionControl.sceneLabel
        color: "#91a7bd"
        font.pixelSize: 13
      }

      Rectangle {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        height: 58
        radius: 8
        color: "#15283b"
        border.color: "#2b4965"
        Column {
          anchors.centerIn: parent
          spacing: 3
          Label { anchors.horizontalCenter: parent.horizontalCenter; text: "任务状态"; color: "#91a7bd"; font.pixelSize: 11 }
          Label {
            anchors.horizontalCenter: parent.horizontalCenter
            text: root.value("state", SanitationMissionControl.missionState)
            color: root.value("state", "").indexOf("FAIL") >= 0 ? "#ff6b6b" : "#57e0b5"
            font.pixelSize: 18
            font.bold: true
          }
        }
      }

      GridLayout {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        columns: 2
        rowSpacing: 8
        columnSpacing: 8
        Button { Layout.fillWidth: true; text: "▶ 开始"; highlighted: true; onClicked: SanitationMissionControl.StartMission() }
        Button { Layout.fillWidth: true; text: "Ⅱ 暂停"; onClicked: SanitationMissionControl.PauseMission() }
        Button { Layout.fillWidth: true; text: "↻ 继续"; onClicked: SanitationMissionControl.ResumeMission() }
        Button { Layout.fillWidth: true; text: "■ 停止任务"; onClicked: SanitationMissionControl.StopMission() }
      }

      Label {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        wrapMode: Text.WordWrap
        text: SanitationMissionControl.operatorMessage
        color: "#d9e4ef"
        font.pixelSize: 12
      }

      RowLayout {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        Label { text: "实时作业地图"; color: "#f4f7fb"; font.pixelSize: 15; font.bold: true }
        Item { Layout.fillWidth: true }
        Label { text: root.value("simulation_speed", "--"); color: "#ffc857"; font.pixelSize: 12; font.bold: true }
      }

      RowLayout {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        spacing: 6
        CheckBox { text: "规划"; checked: root.showPlanned; onToggled: { root.showPlanned = checked; mapCanvas.requestPaint() } }
        CheckBox { text: "实际"; checked: root.showActual; onToggled: { root.showActual = checked; mapCanvas.requestPaint() } }
        CheckBox { text: "补扫"; checked: root.showRepairs; onToggled: { root.showRepairs = checked; mapCanvas.requestPaint() } }
      }

      Canvas {
        id: mapCanvas
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        height: 225

        onPaint: {
          var ctx = getContext("2d")
          ctx.clearRect(0, 0, width, height)
          ctx.fillStyle = "#101f2e"
          ctx.fillRect(0, 0, width, height)
          var boundary = root.value("boundary", [])
          if (!boundary || boundary.length < 3) {
            ctx.fillStyle = "#91a7bd"
            ctx.font = "13px sans-serif"
            ctx.fillText("等待清扫遥测数据…", 16, 30)
            return
          }
          var fieldBoundary = root.value("field_boundary", boundary)
          var minX = fieldBoundary[0][0], maxX = fieldBoundary[0][0]
          var minY = fieldBoundary[0][1], maxY = fieldBoundary[0][1]
          for (var i = 1; i < fieldBoundary.length; ++i) {
            minX = Math.min(minX, fieldBoundary[i][0]); maxX = Math.max(maxX, fieldBoundary[i][0])
            minY = Math.min(minY, fieldBoundary[i][1]); maxY = Math.max(maxY, fieldBoundary[i][1])
          }
          var pad = 16
          var sx = (width - pad * 2) / Math.max(0.1, maxX - minX)
          var sy = (height - pad * 2) / Math.max(0.1, maxY - minY)
          var scale = Math.min(sx, sy)
          function px(x) { return pad + (x - minX) * scale }
          function py(y) { return height - pad - (y - minY) * scale }

          ctx.fillStyle = "#122334"
          ctx.beginPath(); ctx.moveTo(px(fieldBoundary[0][0]), py(fieldBoundary[0][1]))
          for (i = 1; i < fieldBoundary.length; ++i) ctx.lineTo(px(fieldBoundary[i][0]), py(fieldBoundary[i][1]))
          ctx.closePath(); ctx.fill()
          ctx.strokeStyle = "#f58c14"; ctx.lineWidth = 3; ctx.stroke()
          ctx.fillStyle = "#1b3044"
          ctx.beginPath(); ctx.moveTo(px(boundary[0][0]), py(boundary[0][1]))
          for (i = 1; i < boundary.length; ++i) ctx.lineTo(px(boundary[i][0]), py(boundary[i][1]))
          ctx.closePath(); ctx.fill()
          ctx.strokeStyle = "#14d0ff"; ctx.lineWidth = 2.5; ctx.stroke()

          var cells = root.value("cleaned_cells", [])
          var cellSize = Number(root.value("cell_size_m", 0.2)) * scale
          ctx.fillStyle = "rgba(41, 214, 151, 0.65)"
          for (i = 0; i < cells.length; ++i) ctx.fillRect(px(cells[i][0]) - cellSize / 2, py(cells[i][1]) - cellSize / 2, cellSize + 0.5, cellSize + 0.5)

          function line(points, color, lineWidth, dashed) {
            if (!points || points.length < 2) return
            ctx.setLineDash(dashed ? [6, 4] : [])
            ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.beginPath()
            ctx.moveTo(px(points[0][0]), py(points[0][1]))
            for (var j = 1; j < points.length; ++j) ctx.lineTo(px(points[j][0]), py(points[j][1]))
            ctx.stroke(); ctx.setLineDash([])
          }
          var paths = root.value("paths", {})
          var layer, j
          if (root.showPlanned) {
            layer = paths.planned_swaths || []
            for (j = 0; j < layer.length; ++j) line(layer[j], "#ffc857", 2.2, false)
            layer = paths.planned_connectors || []
            for (j = 0; j < layer.length; ++j) line(layer[j], "#aab7c4", 1.6, true)
            line(paths.current_component || root.value("planned_path", []), "#ffffff", 3.0, false)
          }
          if (root.showRepairs) {
            layer = paths.planned_repairs || []
            for (j = 0; j < layer.length; ++j) line(layer[j], "#d77cff", 2.2, true)
            layer = paths.actual_repair || []
            for (j = 0; j < layer.length; ++j) line(layer[j], "#c084fc", 3.0, false)
          }
          if (root.showActual) {
            layer = paths.actual_transit || []
            for (j = 0; j < layer.length; ++j) line(layer[j], "#ff9f43", 1.8, true)
            layer = paths.actual_cleaning || []
            for (j = 0; j < layer.length; ++j) line(layer[j], "#55d6ff", 2.8, false)
            if (!layer.length) line(root.value("trajectory", []), "#55d6ff", 2.8, false)
          }

          var targets = root.value("targets", [])
          for (i = 0; i < targets.length; ++i) {
            ctx.beginPath(); ctx.arc(px(targets[i].x), py(targets[i].y), 5, 0, Math.PI * 2)
            ctx.fillStyle = targets[i].cleaned ? "#29d697" : "#ff6b6b"; ctx.fill()
          }
          var robot = root.value("robot", null)
          if (robot) {
            var rx = px(robot.x), ry = py(robot.y)
            ctx.fillStyle = "#ffffff"; ctx.beginPath(); ctx.arc(rx, ry, 6, 0, Math.PI * 2); ctx.fill()
            ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(rx, ry)
            ctx.lineTo(rx + 12 * Math.cos(robot.yaw), ry - 12 * Math.sin(robot.yaw)); ctx.stroke()
          }
        }
      }

      ProgressBar {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        from: 0; to: 100
        value: Number(root.value("progress_percent", 0))
      }
      RowLayout {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        Label { text: "已清扫 " + root.number("progress_percent", 1, "%"); color: "#57e0b5"; font.bold: true }
        Item { Layout.fillWidth: true }
        Label { text: root.number("cleaned_area_m2", 1, "") + " / " + root.number("total_area_m2", 1, " m²"); color: "#c6d5e3" }
      }

      GridLayout {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        columns: 2
        columnSpacing: 8
        rowSpacing: 8
        Repeater {
          model: [
            ["目标清除", root.value("targets_cleaned", 0) + " / " + root.value("targets_total", 0)],
            ["清扫效率", root.number("cleaning_rate_m2_min", 2, " m²/min")],
            ["累计里程", root.number("distance_m", 1, " m")],
            ["当前速度", root.number("speed_mps", 2, " m/s")],
            ["仿真用时", root.number("elapsed_sim_sec", 0, " s")],
            ["作业步骤", root.value("completed_components", 0) + " / " + root.value("expected_components", 0)]
          ]
          Rectangle {
            Layout.fillWidth: true
            height: 52
            radius: 6
            color: "#15283b"
            border.color: "#233f58"
            Column {
              anchors.centerIn: parent
              spacing: 2
              Label { anchors.horizontalCenter: parent.horizontalCenter; text: modelData[0]; color: "#8299ae"; font.pixelSize: 10 }
              Label { anchors.horizontalCenter: parent.horizontalCenter; text: modelData[1]; color: "#f4f7fb"; font.pixelSize: 14; font.bold: true }
            }
          }
        }
      }

      Label {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        text: "图例  ▰ 橙色外框：外部任务区  ▰ 青色内框：实际清扫区\n■ 绿色：已清扫  ━ 紫色：规划路径  ━ 蓝色：实际轨迹  ● 红色：待清目标\n覆盖率只统计青色内框；数据依据 Gazebo 真值和刷盘足迹，仅用于仿真评估。"
        wrapMode: Text.WordWrap
        color: "#7890a6"
        font.pixelSize: 10
      }
      Button {
        Layout.fillWidth: true
        Layout.leftMargin: 16
        Layout.rightMargin: 16
        text: "关闭 Gazebo"
        onClicked: SanitationMissionControl.CloseGazebo()
      }
      Item { Layout.preferredHeight: 10 }
    }
  }
}
