#include "rviz_plugin/hpp_plugin.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QString>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace hpp_plugin {

// ===========================================================================
//  Destructor
// ===========================================================================
OrchestratorPannel::~OrchestratorPannel() {}

// ===========================================================================
//  Constructor
// ===========================================================================
OrchestratorPannel::OrchestratorPannel(QWidget* parent)
    : rviz_common::Panel(parent) {
  // == Action type =========================================================
  task_combo_ = new QComboBox(this);
  task_combo_->addItem("pick");
  task_combo_->addItem("place");

  // == Buttons =============================================================
  send_btn_ = new QPushButton("▶  Send Goal", this);
  cancel_btn_ = new QPushButton("✖  Cancel Goal", this);
  cancel_btn_->setEnabled(false);

  send_btn_->setStyleSheet(
      "QPushButton { background-color: #2e7d32; color: white; "
      "border-radius: 4px; padding: 6px 12px; font-weight: bold; }"
      "QPushButton:hover { background-color: #43a047; }"
      "QPushButton:disabled { background-color: #555; color: #999; }");

  cancel_btn_->setStyleSheet(
      "QPushButton { background-color: #c62828; color: white; "
      "border-radius: 4px; padding: 6px 12px; font-weight: bold; }"
      "QPushButton:hover { background-color: #e53935; }"
      "QPushButton:disabled { background-color: #555; color: #999; }");

  QHBoxLayout* btn_layout = new QHBoxLayout;
  btn_layout->addWidget(send_btn_);
  btn_layout->addWidget(cancel_btn_);

  // == Log view ============================================================
  log_view_ = new QTextEdit(this);
  log_view_->setReadOnly(true);
  log_view_->setMaximumHeight(160);
  log_view_->setStyleSheet(
      "background-color: #1e1e1e; color: #d4d4d4; "
      "font-family: monospace; font-size: 11px; border-radius: 4px;");

  // == Form group ==========================================================
  QGroupBox* form_box =
      new QGroupBox("PlanBarPick — /orchestrator/plan_bar_handling");
  form_box->setStyleSheet("QGroupBox { font-weight: bold; }");

  QVBoxLayout* form_layout = new QVBoxLayout;
  form_layout->addWidget(new QLabel("Action type:"));
  form_layout->addWidget(task_combo_);  // ← ajouté
  form_layout->addLayout(btn_layout);
  form_box->setLayout(form_layout);

  // == Root layout =========================================================
  QVBoxLayout* root = new QVBoxLayout;
  root->addWidget(form_box);
  root->addWidget(new QLabel("Log:"));
  root->addWidget(log_view_);
  setLayout(root);

  // == Button connections (lambda — no Q_OBJECT needed) ====================
  QObject::connect(send_btn_, &QPushButton::clicked, [this]() { sendGoal(); });
  QObject::connect(cancel_btn_, &QPushButton::clicked,
                   [this]() { cancelGoal(); });
}

// ===========================================================================
//  onInitialize
// ===========================================================================
void OrchestratorPannel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  action_client_ = rclcpp_action::create_client<PlanBarPick>(
      node_, "/orchestrator/plan_bar_handling");

  log("Plugin initialized. Action client created.", "#8bc34a");
}

// ===========================================================================
//  sendGoal
// ===========================================================================
void OrchestratorPannel::sendGoal() {
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    log("✖ Action server not available!", "#ef5350");
    return;
  }

  PlanBarPick::Goal goal;
  goal.task = task_combo_->currentText().toStdString();
  goal.gripper = "tiago_pro/left";
  goal.handle = "reinforcement_bar/left";

  log(QString("▶ Sending: task=%1").arg(task_combo_->currentText()), "#29b6f6");

  auto opts = rclcpp_action::Client<PlanBarPick>::SendGoalOptions();

  opts.goal_response_callback = [this](GoalHandlePBG::SharedPtr handle) {
    goalResponseCallback(handle);
  };

  opts.feedback_callback =
      [this](GoalHandlePBG::SharedPtr h,
             std::shared_ptr<const PlanBarPick::Feedback> fb) {
        feedbackCallback(h, fb);
      };

  opts.result_callback = [this](const GoalHandlePBG::WrappedResult& r) {
    resultCallback(r);
  };

  action_client_->async_send_goal(goal, opts);
  setUiBusy(true);
}

// ===========================================================================
//  cancelGoal
// ===========================================================================
void OrchestratorPannel::cancelGoal() {
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
    log("⚠ Cancel requested.", "#ffa726");
  }
}

// ===========================================================================
//  Action callbacks
// ===========================================================================
void OrchestratorPannel::goalResponseCallback(GoalHandlePBG::SharedPtr handle) {
  if (!handle) {
    QMetaObject::invokeMethod(
        this,
        [this]() {
          log("✖ Goal rejected by server.", "#ef5350");
          setUiBusy(false);
        },
        Qt::QueuedConnection);
    return;
  }
  goal_handle_ = handle;
  QMetaObject::invokeMethod(
      this,
      [this]() { log("✔ Goal accepted. Waiting for result…", "#8bc34a"); },
      Qt::QueuedConnection);
}

void OrchestratorPannel::feedbackCallback(
    GoalHandlePBG::SharedPtr,
    std::shared_ptr<const PlanBarPick::Feedback> feedback) {
  QString msg = QString("↻ status: %1  |  elapsed: %2 s")
                    .arg(QString::fromStdString(feedback->current_action))
                    .arg(QString::fromStdString(feedback->message));

  QMetaObject::invokeMethod(
      this, [this, msg]() { log(msg, "#ce93d8"); }, Qt::QueuedConnection);
}

void OrchestratorPannel::resultCallback(
    const GoalHandlePBG::WrappedResult& result) {
  QString msg;
  QString color;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      msg = "✔ SUCCEEDED";
      color = "#8bc34a";
      break;
    case rclcpp_action::ResultCode::ABORTED:
      msg = "✖ ABORTED";
      color = "#ef5350";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      msg = "⚠ CANCELED";
      color = "#ffa726";
      break;
    default:
      msg = "? UNKNOWN";
      color = "#90a4ae";
      break;
  }

  msg +=
      QString("  |  success=%1").arg(result.result->success ? "true" : "false");

  const QString detail = QString::fromStdString(result.result->message);
  if (!detail.isEmpty()) msg += "  |  " + detail;

  QMetaObject::invokeMethod(
      this,
      [this, msg, color]() {
        log(msg, color);
        setUiBusy(false);
        goal_handle_.reset();
      },
      Qt::QueuedConnection);
}

// ===========================================================================
//  Helpers
// ===========================================================================
void OrchestratorPannel::log(const QString& msg, const QString& color) {
  const QString ts = QDateTime::currentDateTime().toString("hh:mm:ss");
  log_view_->append(QString("<span style='color:#607d8b'>[%1]</span> "
                            "<span style='color:%2'>%3</span>")
                        .arg(ts)
                        .arg(color)
                        .arg(msg));
}

void OrchestratorPannel::setUiBusy(bool busy) {
  send_btn_->setEnabled(!busy);
  cancel_btn_->setEnabled(busy);
  task_combo_->setEnabled(!busy);
}

}  // namespace hpp_plugin

PLUGINLIB_EXPORT_CLASS(hpp_plugin::OrchestratorPannel, rviz_common::Panel)
