#include <iostream>

#include <getopt.h>
#include <stdlib.h> //atoi

#include <tools/CamacAddressParser.h>
#include <tools/CamacErrorPrinter.h>
#include <ADC333.h>
#include <tools/lam_wait.h>
#include <stdio.h>


#define HANDLE_ERROR(x, message) {try { x; } catch(ADC333::CamacError & e) {cerr << message << ": " << e.what() << endl; return 8;}}
using namespace std;
int main(int argc, char * argv[]) {
	CamacAddressParser address("k/0/0/0");
	bool channels[ADC333::CHAN_COUNT] = {false};
	bool cycle = false, manual = false;
	int opt, gain = 0;
	ADC333 module;
	while ((opt = getopt(argc, argv, "hrmc:t:a:g:")) != -1) {
		switch(opt) {
			case 'a':	
				if (address.parse(optarg)) {
					cerr << "Invalid camac address: " << optarg << endl;
					return 1;
				}
				break;
			case 'c':
			{
				int chan = atoi(optarg);
				if (chan < 1 || chan > ADC333::CHAN_COUNT) {
					cerr << "Channel is out of range" << endl;
					return 4;
				}
				channels[chan-1] = true;
				break;
			}
			case 't':
			{
				int period = atoi(optarg);
				try {
					module.SetTickInNanoSeconds(period);
				} catch(...) {
					cerr <<
					"Wrong period: " << period << 
					"\nAllowed periods: 500, 1000, 2000, 4000, 8000, 16000, 32000, 0(for external clock)" 
					<< endl;
					return 10;
				}
				break;
			}
			case 'r':
				cycle = true;
				break;
			case 'm':
				manual = true;
				break;
			case 'g': {
				int tmp = atoi(optarg);
				if (tmp < 0 || tmp > 3) {
					cerr << "Gain should be in 0..3" << endl;
					return 4;
				}
				gain = tmp;
				break;
			}
			default:
				cout <<
				argv[0] <<
				" -a address [-h] [-c number [-c number ...]]\n"
				"Options:\n"
				" -m         - manual trigger. Press Enter to emulate trigger.\n"
				" -a address - use this address\n"
				" -r         - cycle mode. Readout stops on signal.\n"
				" -g [0..3]  - gain mode.\n"
				" -c number  - enable a channel [1.."<< ADC333::CHAN_COUNT << "]\n"
				" -h         - show help\n"
				"In normal mode program waits for the hardware trigger, performs the measurement and dumps the data.\n"
				"In cycle mode program measures in cycle, stops on the hardware trigger and dumps latest data measured."
				<< endl;
			return 0;
		}

	}

	try {
	if (module.Bind(address.address()) < 0) {
		cerr << "Module bind failed for address " << address << endl;
		return 2;
	}

	int rv = module.Init();
	if (rv & CAMAC_CC_ERRORS) {
		cerr << "Module init failed" << CamacErrorPrinter(rv) << endl;
		return 3;
	}

	{
		unsigned gains[ADC333::CHAN_COUNT];
		for (int i = 0; i < ADC333::CHAN_COUNT; ++i) {
			if (channels[i])
				gains[i] = gain + 1;
			else
				gains[i] = 0;
		}
		module.EnableChannels(gains);
	}

//	lam_complete(module);
	if (cycle) {
		HANDLE_ERROR(module.StartCycle(), "Failed to start cycle measurement");
		if (manual) {
			clog << "Press any key to stop measurement" << endl;
			getchar();
			module.Stop();
		}
	} else {
		HANDLE_ERROR(module.StartSingleRun(), "Failed to start measurement");
		if (manual) {
			HANDLE_ERROR(module.Trigger(), "Software trigger failed");
		}
	}
	while(true) {
		df_timeout_t timeout = 1*1000;
		rv = lam_wait(module, &timeout);

		if (rv & CAMAC_CC_ERRORS) {
			if (rv & CAMAC_CC_NOT_Q) {
				break;
			}
			cerr << "Error while waiting for LAM: " << CamacErrorPrinter(rv) << endl;
			return 5;
		}
		
		if (rv & CAMAC_CC_BOOL) {
			if (!module.CheckLAM()) {
				cerr << "False LAM positive detected" << endl;
				continue;
			}
			break;
		}
		if (module.CheckLAM()) {
			cerr << "False LAM timeout detected " << endl;
		} else {
			clog << "Still wating LAM" <<endl;
		}
	}

	vector<double> data[ADC333::CHAN_COUNT];
	{
		for (unsigned chIdx = 0; chIdx < ADC333::CHAN_COUNT; chIdx++) {
			if (channels[chIdx]) {
				HANDLE_ERROR(module.Read(chIdx, data[chIdx]), "Failed to read channel");
			}
		}
	}
	unsigned i = 0;
	while(true) {
		bool end = false;
		bool first = true;
		for (unsigned chIdx = 0; chIdx < ADC333::CHAN_COUNT; chIdx++) {
			if (channels[chIdx]) {
				if (i >= data[chIdx].size()) {
					end = true;
					break;
				}
				if (!first)
					cout << "\t";
				first = false;
				cout << data[chIdx][i];
			}
		}
		cout << "\n";
		if (!cout) {
			cerr << "Output fail" << endl;
			return 10;
		}
		++i;
		if (end)
			break;
	}
	} catch(ADC333::CamacError & e) {
		cerr << e.what() << endl;
		return 1;
	}
	cout.flush();
	return 0;
}
