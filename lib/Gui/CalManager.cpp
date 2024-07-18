/*
 * Copyright (c) 2016, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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