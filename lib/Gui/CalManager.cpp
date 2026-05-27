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

#include "CalManager.h"

CalManager::CalManager(QWidget *parent)
{
	SyncToolsCal::Shared::startNewWinMutex.lock();
	SyncToolsCal::Shared::startNewWin = false;
	SyncToolsCal::Shared::startNewWinMutex.unlock();
	SyncToolsCal::Shared::startNewWinMutex.lock();
	SyncToolsCal::Shared::stopWin = false;
	SyncToolsCal::Shared::startNewWinMutex.unlock();
	connect(&_winTimer, SIGNAL(timeout()), this, SLOT(thread()));
	_winTimer.start(1000); // 10Hz
}

CalManager::~CalManager()
{
// 	mutex_.lock();
	for(std::vector<MainWindow*>::iterator it = mainWin_.begin(); it != mainWin_.end(); it++)
	{
		(*it)->stop();
		delete *it;
	}
// 	mutex_.unlock();
}

void CalManager::StartNewWindows()
{
// 	mutex_.lock();
// 	COUT << "create new windows" << std::endl << std::flush;
	if(mainWin_.size() >= 1)
		mainWin_.back()->stopRefresh();
	mainWin_.push_back(new MainWindow(executionSimulatedAnnealing_));
	mainWin_.back()->show();
	
// 	COUT << "create end windows" << std::endl << std::flush;
// 	mutex_.unlock();
}

void CalManager::stopPreviousWindows()
{
// 	mutex_.lock();
// 	if(mainWin_.size() > 1)
// 		mainWin_.at(mainWin_.size() - 1)->stopRefresh();
// 	else
// 		mainWin_.back()->stopRefresh();
// 	mutex_.unlock();
}


void CalManager::thread()
{
// 	COUT << "ok" << std::endl;
	SyncToolsCal::Shared::endGuiMutex.lock();
	if(SyncToolsCal::Shared::endGui)
	{
		this->close();
		_winTimer.stop();
	}
	SyncToolsCal::Shared::endGuiMutex.unlock();
	SyncToolsCal::Shared::startNewWinMutex.lock();
	if(SyncToolsCal::Shared::startNewWin)
	{
// 		COUT << "inside" << std::endl;
		SyncToolsCal::Shared::startNewWin = false;
		StartNewWindows();
	}
	SyncToolsCal::Shared::startNewWinMutex.unlock();
	SyncToolsCal::Shared::startNewWinMutex.lock();
	if(SyncToolsCal::Shared::stopWin)
	{
		SyncToolsCal::Shared::stopWin = false;
		stopPreviousWindows();
	}
	SyncToolsCal::Shared::startNewWinMutex.unlock();
}