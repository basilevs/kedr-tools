#include <iostream>

#include <unistd.h>

#include <getopt.h>
#include <stdlib.h> //atoi

#include <tools/CamacAddressParser.h>
#include <tools/CamacErrorPrinter.h>
#include <ZIF_L0323.h>
#include <IntegralVoltmeter_C0309.h>
#include <Discriminator_D0302.h>
#include <ADC333.h>
#include <tools/lam_wait.h>
#include <stdio.h>


#define HANDLE_ERROR(x, message) {try { x; } catch(ADC333::CamacError & e) {cerr << message << ": " << e.what() << endl; return 8;}}

#define HANDLE_CAMAC_CODE(code, message) {int x = (code); if (x & CAMAC_CC_ERRORS) {cerr << message << ": " << CamacErrorPrinter(x) << endl; return 3;}}
using namespace std;


int main(int argc, char * argv[]) {
	CamacAddressParser adcAddress("k0607-lsi6/0/0/16");
	CamacAddressParser qdcAddress("k0607-lsi6/0/0/21");
	CamacAddressParser shaperAddress("k0607-lsi6/0/0/20");
	CamacAddressParser discrAddress("k0607-lsi6/0/0/22");
	CamacAddressParser & address = adcAddress;

	bool channels[ADC333::CHAN_COUNT] = {false};
	channels[0]=true;
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
	
	{	// Configure discriminator
		Discriminator_D0302 d;
		if (d.Bind(discrAddress.address()) < 0) {
			cerr << "Failed to bind to discriminator on address " <<  discrAddress << endl;
			return 2;
		}
		HANDLE_CAMAC_CODE(d.Init(), "Failed to init discriminator " << discrAddress);
		const u16_t threshold = 1;
		HANDLE_CAMAC_CODE(d.SetThreshold(threshold, 1), "Failed to set discriminator threshold");
		u16_t data;		
		HANDLE_CAMAC_CODE(d.GetThreshold(&data, 1), "Failed to verify threshold");
//		assert(data == threshold);
//		HANDLE_CAMAC_CODE(d.Verify(), "Failed to verify discriminator settings");
	}
	{ // Configure shaper
		ZIF_L0323 shaper;
		if (shaper.Bind(shaperAddress.address()) < 0) {
			cerr << "Failed to bind to shaper " <<  shaperAddress << endl;
			return 2;
		}
		HANDLE_CAMAC_CODE(shaper.Init(), "Failed to init shaper " << discrAddress);
		HANDLE_CAMAC_CODE(shaper.SetDuration(0, 1), "Failed to set discriminator threshold");		
	}
	

	try {
	if (module.Bind(address.address()) < 0) {
		cerr << "Module bind failed for address " << address << endl;
		return 2;
	}

	int rv = module.Init();
	if (rv & CAMAC_CC_ERRORS) {
		cerr << "Module init failed " << CamacErrorPrinter(rv) << endl;
		return 3;
	}
	module.Reset();

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
	
//	module.SetTickInNanoSeconds(32000);
	module.SetTickInNanoSeconds(0);
	IV_C0309 qdc;
	{
		if (qdc.Bind(qdcAddress.address()) < 0) {
			cerr << "Failed to bind to qdc" << endl;
			return 2;
		}
		if (qdc.Init() & CAMAC_CC_ERRORS) {
			cerr << "Failed to init qdc" << endl;
			return 3;
		}
		qdc.ClearLAM();
		qdc.SetControl(0, 0);
	}
	HANDLE_ERROR(module.StartCycle(), "Failed to start cycle measurement");
	while (true) {
		df_timeout_t timeout = 1*1000;	
		qdc.Stop();
		HANDLE_ERROR(qdc.CheckLAM() ^ CAMAC_CC_NOT_Q, "Unexpected LIPA LAM");
		qdc.Start();
		int rv = lam_wait(qdc, &timeout);
		if (rv & CAMAC_CC_ERRORS) {
			cerr << "Failed to wait LAM from QDC" << endl;
			break;
		}
		rv = qdc.CheckLAM();
		if (rv & CAMAC_CC_ERRORS) {
			cerr << "Failed to check LAM from QDC" << endl;
			break;
		}
		double voltage = 0;
		HANDLE_ERROR(qdc.ReadVoltage(voltage), "Failed to read voltage from LIPA");
		if (abs(voltage) > 30) {
			cerr << "Threshold reached in LIPA. Voltage: " << voltage << endl;
			cout << "#LIPA\t" << voltage << "\n";
			break;
		}		
	}
	sleep(10);
	module.Stop();
	cout << "#STOPTIME\t" << time(0) << "\n";
	
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
	cout << "#END" << "\n";
	cout.flush();
	return 0;
}
