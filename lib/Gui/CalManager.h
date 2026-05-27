// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software. Any changes to this code, should be shared back in the open repository: https://github.com/CEINMS-RT. See license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE.
//
// The methodologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots. TechRxiv. DOI: 10.36227/techrxiv.173397962.28177284/v1"
//

#ifndef CALMANAGER_H
#define CALMANAGER_H

#include "mainwindowCal.h"

class CalManager:public QMainWindow
{
	Q_OBJECT
	public:
		CalManager(QWidget *parent = 0);
		~CalManager();
		void StartNewWindows();
		void stopPreviousWindows();
	inline void setSimAnnExec(const ExecutionSimulatedAnnealing& executionSimulatedAnnealing)
	{
		executionSimulatedAnnealing_ = &executionSimulatedAnnealing;
	}
protected:
	boost::mutex mutex_;
	std::vector<MainWindow*> mainWin_;
	const ExecutionSimulatedAnnealing* executionSimulatedAnnealing_;
	
	QTimer _winTimer;
	private slots:
	void thread();
};

#endif // CALMANAGER_H
