/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "DoubleSolenoid.h"
#include "WPIErrors.h"
#include <string.h>
#include "LiveWindow/LiveWindow.h"

/**
 * Constructor.
 *
 * @param forwardChannel The forward channel on the module to control.
 * @param reverseChannel The reverse channel on the module to control.
 */
DoubleSolenoid::DoubleSolenoid(uint32_t forwardChannel, uint32_t reverseChannel)
  : DoubleSolenoid(1, forwardChannel, reverseChannel) {}

/**
 * Constructor.
 *
 * @param moduleNumber The solenoid module (1 or 2).
 * @param forwardChannel The forward channel on the module to control.
 * @param reverseChannel The reverse channel on the module to control.
 */
DoubleSolenoid::DoubleSolenoid(uint8_t moduleNumber, uint32_t forwardChannel, uint32_t reverseChannel)
{
    m_reversed = false;
    if (reverseChannel < forwardChannel) { // Swap ports to get the right address
        int channel = reverseChannel;
        reverseChannel = forwardChannel;
        forwardChannel = channel;
        m_reversed = true;
    }
    char buffer[50];
    int n = sprintf(buffer, "pneumatic/%d/%d/%d/%d", moduleNumber,
                    forwardChannel, moduleNumber, reverseChannel);
    m_impl = new SimContinuousOutput(buffer);  
  
	LiveWindow::GetInstance()->AddActuator("DoubleSolenoid", moduleNumber,
                                           forwardChannel, this);
}

/**
 * Set the value of a solenoid.
 *
 * @param value Move the solenoid to forward, reverse, or don't move it.
 */
void DoubleSolenoid::Set(Value value)
{
    m_value = value;
	switch(value)
	{
	case kOff:
        m_impl->Set(0);
        break;
	case kForward:
        m_impl->Set(m_reversed ? -1 : 1);
		break;
	case kReverse:
        m_impl->Set(m_reversed ? 1 : -1);
		break;
	}
}

/**
 * Read the current value of the solenoid.
 *
 * @return The current value of the solenoid.
 */
DoubleSolenoid::Value DoubleSolenoid::Get() const
{
	return m_value;
}

void DoubleSolenoid::ValueChanged(std::shared_ptr<ITable> source, const std::string& key, EntryValue value, bool isNew) {
	Value lvalue = kOff;
	std::string *val = (std::string *)value.ptr;
	if (*val == "Forward")
		lvalue = kForward;
	else if (*val == "Reverse")
		lvalue = kReverse;
	Set(lvalue);
}

void DoubleSolenoid::UpdateTable() {
	if (m_table != nullptr) {
		m_table->PutString("Value", (Get() == kForward ? "Forward" : (Get() == kReverse ? "Reverse" : "Off")));
	}
}

void DoubleSolenoid::StartLiveWindowMode() {
	Set(kOff);
	if (m_table != nullptr) {
		m_table->AddTableListener("Value", this, true);
	}
}

void DoubleSolenoid::StopLiveWindowMode() {
	Set(kOff);
	if (m_table != nullptr) {
		m_table->RemoveTableListener(this);
	}
}

std::string DoubleSolenoid::GetSmartDashboardType() const {
	return "Double Solenoid";
}

void DoubleSolenoid::InitTable(std::shared_ptr<ITable> subTable) {
	m_table = subTable;
	UpdateTable();
}

std::shared_ptr<ITable> DoubleSolenoid::GetTable() const {
	return m_table;
}
