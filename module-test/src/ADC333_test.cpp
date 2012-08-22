#include <iostream>

#include <getopt.h>
#include <stdlib.h> //atoi

#include <tools/CamacAddressParser.h>
#include <tools/CamacErrorPrinter.h>
#include <ADC333.h>
#include <tools/lam_wait.h>



#define HANDLE_ERROR(x, message) {int a = x; if (a & CAMAC_CC_ERRORS) cerr << message << ": " << CamacErrorPrinter(a) << endl; return 8;}
using namespace std;
int main(int argc, char * argv[]) {
	CamacAddressParser address("k/0/0/0");
	bool channels[ADC333::CHAN_COUNT] = {false};
	bool cycle = false, manual = false;
	unsigned gain = 0;

	int opt;
	while ((opt = getopt(argc, argv, "hrmca:g:")) != -1) {
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
			}
			default:
				cout <<
				argv[0] <<
				" -a address [-h] [-c number [-c number ...]]" <<
				"Options:\n"
				"-m          - manual trigger. Press Enter to emulate trigger.\n"
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

	ADC333 module;
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

	if (cycle) {
		HANDLE_ERROR(module.StartCycle(), "Failed to start cycle measurement");
		if (manual) {
			clog << "Press any key to stop measurement" << endl;
			char ch;
			cin >> ch;
			module.Stop();
		}
	} else {
		HANDLE_ERROR(module.StartSingleRun(), "Failed to start measurement");
		if (manual) {
			clog << "Press any key to start measurement" << endl;
			char ch;
			cin >> ch;
			module.Trigger();
		}
	}

	while(true) {
		df_timeout_t timeout = 100*1000;
		rv = lam_wait(module, &timeout);

		if (rv & CAMAC_CC_ERRORS) {
			cerr << "Error while waiting for LAM: " << CamacErrorPrinter(rv) << endl;
			return 5;
		}

		if (rv & CAMAC_CC_BOOL) {
			break;
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
				if (!first) {
					cout << "\t";
				}
				first = false;
				cout << data[chIdx][i];
			}
		}
		cout << "\n";
		++i;
	}

	return 0;
}
