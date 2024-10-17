// This source code is part of:
// "Calibrated EMG-Informed Neuromusculoskeletal Modeling (CEINMS) Toolbox". 
// Copyright (C) 2015 David G. Lloyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato
//
// CEINMS is not free software. You can not redistribute it without the consent of the authors.
// The recipient of this software shall provide the authors with a full set of software, 
// with the source code, and related documentation in the vent of modifications and/or additional developments to CEINMS. 
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
// Sartori M., Reggiani M., Farina D., Lloyd D.G., (2012) "EMG-Driven Forward-Dynamic Estimation of Muscle Force and Joint Moment about Multiple Degrees of Freedom in the Human Lower Extremity," PLoS ONE 7(12): e52618. doi:10.1371/journal.pone.0052618
// Sartori M., Farina D., Lloyd D.G., (2014) “Hybrid neuromusculoskeletal modeling to best track joint moments using a balance between muscle excitations derived from electromyograms and optimization,” J. Biomech., vol. 47, no. 15, pp. 3613–3621, 
//
// Please, contact the authors to receive a copy of the "non-disclosure" and "material transfer" agreements:
// email: david.lloyd@griffith.edu.au, monica.reggiani@gmail.com, massimo.srt@gmail.com, claudio.pizzolato.uni@gmail.com 
#ifndef SimpleFileLogger_h
#define SimpleFileLogger_h

#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <stdlib.h>

extern std::string trialNameCommandLine;

namespace Logger {
    enum LogID{
				Activations, 
                FibreLengths, 
                FibreVelocities,
                MuscleForces,
                Torques,
                Emgs,
                AdjustedEmgs
	};


    template <typename NMSmodelT>
    class SimpleFileLogger {


        
    public:
        SimpleFileLogger(NMSmodelT& subject);
        void addLog(LogID logID);
        void log(double time, LogID logID);
        
    private:
        void logDataVector(const std::vector<double>& data, std::ofstream& outFile);
        void initFile(const std::vector<std::string>& names, std::ofstream& outFile);
        NMSmodelT& subject_;
        
        double time_;
        std::string outputDir_;
        std::vector<boost::shared_ptr<std::ofstream> > outFiles_;    
        std::vector<LogID> fileTypes_;
    };


    template <typename NMSmodelT>
    SimpleFileLogger<NMSmodelT>::SimpleFileLogger(NMSmodelT& subject)
    :subject_(subject),
    outputDir_("./Output/"+trialNameCommandLine) {;
    
        boost::filesystem::path dir(outputDir_);
		if(!boost::filesystem::exists(dir)) {
			if(boost::filesystem::create_directories(dir)) {
				std::cout << "Error: Cannot create the output directory " + outputDir_ << std::endl;  
				exit(EXIT_FAILURE);        
			}  
			std::cout << "Created output directory " + dir.string() << std::endl;
		}
		else std::cout << "Using " +  dir.string() + " as output directory\n";
	}
	

    template <typename NMSmodelT>
    void SimpleFileLogger<NMSmodelT>::addLog(LogID logID) {
        
        std::string filename;
        switch(logID) {
            case Activations:
                filename = "activations.csv";
                break;
            case FibreLengths:
                filename = "fiberLengths.csv";
                break;
            case FibreVelocities:
                filename = "fiberVelocities.csv";
                break;
            case MuscleForces:
                filename = "muscleTendonForces.csv";
                break;        
            case Torques:
                filename = "torques.csv";
                break; 
            case Emgs:
                filename = "emgs.csv";
                break; 
            case AdjustedEmgs:
                filename = "adjustedEmgs.csv";
                break; 
        }
        std::string outFilename =  outputDir_+filename;
        boost::shared_ptr<std::ofstream> file_ptr(new std::ofstream(outFilename.c_str()));
        outFiles_.push_back(file_ptr);
        if(!(outFiles_.back()->is_open())) {
            std::cout << "ERROR: " + filename + " cannot be opened!\n";
            exit(EXIT_FAILURE);
        }
        
        fileTypes_.push_back(logID);
        
        std::vector<std::string> names;
        if(logID == Torques)
            subject_.getDoFNames(names);
        else
            subject_.getMuscleNames(names);
        
        initFile(names, *outFiles_.back());
     
    }


    template <typename NMSmodelT>
    void SimpleFileLogger<NMSmodelT>::log(double time, LogID logID) {
    
        time_ = time;
        std::vector<double> data;
        
        switch(logID) {
            case Activations:
                subject_.getActivations(data);
                break;
            case FibreLengths:
                subject_.getFiberLengths(data);
                break;
            case FibreVelocities:
                subject_.getFiberVelocities(data);
                break;
            case MuscleForces:
                subject_.getMuscleForces(data);
                break;        
            case Torques:
                subject_.getTorques(data);
                break; 
            case Emgs:
            case AdjustedEmgs:
                subject_.getEmgs(data);
                break;    
        }

        unsigned dst = std::distance(fileTypes_.begin(), 
                                     std::find(fileTypes_.begin(), fileTypes_.end(), 
                                     logID));
        logDataVector(data, *outFiles_.at(dst));
    }
    
    

    template <typename NMSmodelT>
    void SimpleFileLogger<NMSmodelT>::logDataVector(const std::vector<double>& data, std::ofstream& outFile) {
        
        outFile << std::setprecision (15) << time_ << " ";
        for (unsigned i = 0; i < data.size()-1 ; ++i)
            outFile << data.at(i) << " ";
        outFile << data.back() << std::endl; 
    }


    template <typename NMSmodelT>
    void SimpleFileLogger<NMSmodelT>::initFile(const std::vector<std::string>& names, std::ofstream& outFile) {
        
        outFile << "Time ";
        for (unsigned int i = 0; i < names.size()-1 ; ++i )
            outFile << names.at(i) << " ";
        outFile << names.back() << std::endl;
    }

    
};

#endif
