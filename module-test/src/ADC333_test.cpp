#include <iostream>

#include <getopt.h>
#include <stdlib.h> //atoi

#include <tools/CamacAddressParser.h>
#include <tools/CamacErrorPrinter.h>
#include <ADC333.h>
#include <tools/lam_wait.h>

using namespace std;
int main(int argc, char * argv[]) {
	CamacAddressParser address("k/0/0/0");
	bool channels[ADC333::CHAN_COUNT] = {false};
	bool cycle = false;
	int opt;

	while ((opt = getopt(argc, argv, "hrca:")) != -1) {
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
			default:
				cout <<
				argv[0] <<
				" -a address [-h] [-c number [-c number ...]]" <<
				"Options:\n"
				" -a address - use this address\n"
				" -c number  - enable a channel [1..4]\n"
				" -h         - show help"
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

	module.EnableChannels(channels);

	if (cycle) {
		module.StartCycle();
	} else {
		module.StartSingleRun();
	}

	df_timeout_t timeout = 100*1000;
	rv = lam_wait(module, &timeout);

	if (rv & CAMAC_CC_ERRORS) {
		cerr << "Error while waiting for LAM: " << CamacErrorPrinter(rv) << endl;
		return 5;
	}

	if (!(rv & CAMAC_CC_BOOL)) {
		cerr << "Timeout while waiting for LAM: " << CamacErrorPrinter(rv) << endl;
		return 6;
	}

	vector<double> data[ADC333::CHAN_COUNT];
	{
		for (unsigned chIdx = 0; chIdx < ADC333::CHAN_COUNT; chIdx++) {
			if (channels[chIdx]) {
				module.Read(chIdx, data[chIdx]);
			}
		}
	}
	int i = 0;
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
	}

	return 0;
}
