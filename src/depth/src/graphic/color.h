#pragma once

#include <opencv2/imgproc.hpp>
#include <pleno/types.h>

// Copyright 2019 Google LLC.
// SPDX-License-Identifier: Apache-2.0
// Author: Anton Mikhailov
// The look-up tables contains 256 entries. Each entry is a an sRGB triplet.

constexpr float turbo_srgb_floats[256][3] = {{0.18995,0.07176,0.23217},{0.19483,0.08339,0.26149},{0.19956,0.09498,0.29024},{0.20415,0.10652,0.31844},{0.20860,0.11802,0.34607},{0.21291,0.12947,0.37314},{0.21708,0.14087,0.39964},{0.22111,0.15223,0.42558},{0.22500,0.16354,0.45096},{0.22875,0.17481,0.47578},{0.23236,0.18603,0.50004},{0.23582,0.19720,0.52373},{0.23915,0.20833,0.54686},{0.24234,0.21941,0.56942},{0.24539,0.23044,0.59142},{0.24830,0.24143,0.61286},{0.25107,0.25237,0.63374},{0.25369,0.26327,0.65406},{0.25618,0.27412,0.67381},{0.25853,0.28492,0.69300},{0.26074,0.29568,0.71162},{0.26280,0.30639,0.72968},{0.26473,0.31706,0.74718},{0.26652,0.32768,0.76412},{0.26816,0.33825,0.78050},{0.26967,0.34878,0.79631},{0.27103,0.35926,0.81156},{0.27226,0.36970,0.82624},{0.27334,0.38008,0.84037},{0.27429,0.39043,0.85393},{0.27509,0.40072,0.86692},{0.27576,0.41097,0.87936},{0.27628,0.42118,0.89123},{0.27667,0.43134,0.90254},{0.27691,0.44145,0.91328},{0.27701,0.45152,0.92347},{0.27698,0.46153,0.93309},{0.27680,0.47151,0.94214},{0.27648,0.48144,0.95064},{0.27603,0.49132,0.95857},{0.27543,0.50115,0.96594},{0.27469,0.51094,0.97275},{0.27381,0.52069,0.97899},{0.27273,0.53040,0.98461},{0.27106,0.54015,0.98930},{0.26878,0.54995,0.99303},{0.26592,0.55979,0.99583},{0.26252,0.56967,0.99773},{0.25862,0.57958,0.99876},{0.25425,0.58950,0.99896},{0.24946,0.59943,0.99835},{0.24427,0.60937,0.99697},{0.23874,0.61931,0.99485},{0.23288,0.62923,0.99202},{0.22676,0.63913,0.98851},{0.22039,0.64901,0.98436},{0.21382,0.65886,0.97959},{0.20708,0.66866,0.97423},{0.20021,0.67842,0.96833},{0.19326,0.68812,0.96190},{0.18625,0.69775,0.95498},{0.17923,0.70732,0.94761},{0.17223,0.71680,0.93981},{0.16529,0.72620,0.93161},{0.15844,0.73551,0.92305},{0.15173,0.74472,0.91416},{0.14519,0.75381,0.90496},{0.13886,0.76279,0.89550},{0.13278,0.77165,0.88580},{0.12698,0.78037,0.87590},{0.12151,0.78896,0.86581},{0.11639,0.79740,0.85559},{0.11167,0.80569,0.84525},{0.10738,0.81381,0.83484},{0.10357,0.82177,0.82437},{0.10026,0.82955,0.81389},{0.09750,0.83714,0.80342},{0.09532,0.84455,0.79299},{0.09377,0.85175,0.78264},{0.09287,0.85875,0.77240},{0.09267,0.86554,0.76230},{0.09320,0.87211,0.75237},{0.09451,0.87844,0.74265},{0.09662,0.88454,0.73316},{0.09958,0.89040,0.72393},{0.10342,0.89600,0.71500},{0.10815,0.90142,0.70599},{0.11374,0.90673,0.69651},{0.12014,0.91193,0.68660},{0.12733,0.91701,0.67627},{0.13526,0.92197,0.66556},{0.14391,0.92680,0.65448},{0.15323,0.93151,0.64308},{0.16319,0.93609,0.63137},{0.17377,0.94053,0.61938},{0.18491,0.94484,0.60713},{0.19659,0.94901,0.59466},{0.20877,0.95304,0.58199},{0.22142,0.95692,0.56914},{0.23449,0.96065,0.55614},{0.24797,0.96423,0.54303},{0.26180,0.96765,0.52981},{0.27597,0.97092,0.51653},{0.29042,0.97403,0.50321},{0.30513,0.97697,0.48987},{0.32006,0.97974,0.47654},{0.33517,0.98234,0.46325},{0.35043,0.98477,0.45002},{0.36581,0.98702,0.43688},{0.38127,0.98909,0.42386},{0.39678,0.99098,0.41098},{0.41229,0.99268,0.39826},{0.42778,0.99419,0.38575},{0.44321,0.99551,0.37345},{0.45854,0.99663,0.36140},{0.47375,0.99755,0.34963},{0.48879,0.99828,0.33816},{0.50362,0.99879,0.32701},{0.51822,0.99910,0.31622},{0.53255,0.99919,0.30581},{0.54658,0.99907,0.29581},{0.56026,0.99873,0.28623},{0.57357,0.99817,0.27712},{0.58646,0.99739,0.26849},{0.59891,0.99638,0.26038},{0.61088,0.99514,0.25280},{0.62233,0.99366,0.24579},{0.63323,0.99195,0.23937},{0.64362,0.98999,0.23356},{0.65394,0.98775,0.22835},{0.66428,0.98524,0.22370},{0.67462,0.98246,0.21960},{0.68494,0.97941,0.21602},{0.69525,0.97610,0.21294},{0.70553,0.97255,0.21032},{0.71577,0.96875,0.20815},{0.72596,0.96470,0.20640},{0.73610,0.96043,0.20504},{0.74617,0.95593,0.20406},{0.75617,0.95121,0.20343},{0.76608,0.94627,0.20311},{0.77591,0.94113,0.20310},{0.78563,0.93579,0.20336},{0.79524,0.93025,0.20386},{0.80473,0.92452,0.20459},{0.81410,0.91861,0.20552},{0.82333,0.91253,0.20663},{0.83241,0.90627,0.20788},{0.84133,0.89986,0.20926},{0.85010,0.89328,0.21074},{0.85868,0.88655,0.21230},{0.86709,0.87968,0.21391},{0.87530,0.87267,0.21555},{0.88331,0.86553,0.21719},{0.89112,0.85826,0.21880},{0.89870,0.85087,0.22038},{0.90605,0.84337,0.22188},{0.91317,0.83576,0.22328},{0.92004,0.82806,0.22456},{0.92666,0.82025,0.22570},{0.93301,0.81236,0.22667},{0.93909,0.80439,0.22744},{0.94489,0.79634,0.22800},{0.95039,0.78823,0.22831},{0.95560,0.78005,0.22836},{0.96049,0.77181,0.22811},{0.96507,0.76352,0.22754},{0.96931,0.75519,0.22663},{0.97323,0.74682,0.22536},{0.97679,0.73842,0.22369},{0.98000,0.73000,0.22161},{0.98289,0.72140,0.21918},{0.98549,0.71250,0.21650},{0.98781,0.70330,0.21358},{0.98986,0.69382,0.21043},{0.99163,0.68408,0.20706},{0.99314,0.67408,0.20348},{0.99438,0.66386,0.19971},{0.99535,0.65341,0.19577},{0.99607,0.64277,0.19165},{0.99654,0.63193,0.18738},{0.99675,0.62093,0.18297},{0.99672,0.60977,0.17842},{0.99644,0.59846,0.17376},{0.99593,0.58703,0.16899},{0.99517,0.57549,0.16412},{0.99419,0.56386,0.15918},{0.99297,0.55214,0.15417},{0.99153,0.54036,0.14910},{0.98987,0.52854,0.14398},{0.98799,0.51667,0.13883},{0.98590,0.50479,0.13367},{0.98360,0.49291,0.12849},{0.98108,0.48104,0.12332},{0.97837,0.46920,0.11817},{0.97545,0.45740,0.11305},{0.97234,0.44565,0.10797},{0.96904,0.43399,0.10294},{0.96555,0.42241,0.09798},{0.96187,0.41093,0.09310},{0.95801,0.39958,0.08831},{0.95398,0.38836,0.08362},{0.94977,0.37729,0.07905},{0.94538,0.36638,0.07461},{0.94084,0.35566,0.07031},{0.93612,0.34513,0.06616},{0.93125,0.33482,0.06218},{0.92623,0.32473,0.05837},{0.92105,0.31489,0.05475},{0.91572,0.30530,0.05134},{0.91024,0.29599,0.04814},{0.90463,0.28696,0.04516},{0.89888,0.27824,0.04243},{0.89298,0.26981,0.03993},{0.88691,0.26152,0.03753},{0.88066,0.25334,0.03521},{0.87422,0.24526,0.03297},{0.86760,0.23730,0.03082},{0.86079,0.22945,0.02875},{0.85380,0.22170,0.02677},{0.84662,0.21407,0.02487},{0.83926,0.20654,0.02305},{0.83172,0.19912,0.02131},{0.82399,0.19182,0.01966},{0.81608,0.18462,0.01809},{0.80799,0.17753,0.01660},{0.79971,0.17055,0.01520},{0.79125,0.16368,0.01387},{0.78260,0.15693,0.01264},{0.77377,0.15028,0.01148},{0.76476,0.14374,0.01041},{0.75556,0.13731,0.00942},{0.74617,0.13098,0.00851},{0.73661,0.12477,0.00769},{0.72686,0.11867,0.00695},{0.71692,0.11268,0.00629},{0.70680,0.10680,0.00571},{0.69650,0.10102,0.00522},{0.68602,0.09536,0.00481},{0.67535,0.08980,0.00449},{0.66449,0.08436,0.00424},{0.65345,0.07902,0.00408},{0.64223,0.07380,0.00401},{0.63082,0.06868,0.00401},{0.61923,0.06367,0.00410},{0.60746,0.05878,0.00427},{0.59550,0.05399,0.00453},{0.58336,0.04931,0.00486},{0.57103,0.04474,0.00529},{0.55852,0.04028,0.00579},{0.54583,0.03593,0.00638},{0.53295,0.03169,0.00705},{0.51989,0.02756,0.00780},{0.50664,0.02354,0.00863},{0.49321,0.01963,0.00955},{0.47960,0.01583,0.01055}};

constexpr unsigned char turbo_srgb_bytes[256][3] = {{48,18,59},{50,21,67},{51,24,74},{52,27,81},{53,30,88},{54,33,95},{55,36,102},{56,39,109},{57,42,115},{58,45,121},{59,47,128},{60,50,134},{61,53,139},{62,56,145},{63,59,151},{63,62,156},{64,64,162},{65,67,167},{65,70,172},{66,73,177},{66,75,181},{67,78,186},{68,81,191},{68,84,195},{68,86,199},{69,89,203},{69,92,207},{69,94,211},{70,97,214},{70,100,218},{70,102,221},{70,105,224},{70,107,227},{71,110,230},{71,113,233},{71,115,235},{71,118,238},{71,120,240},{71,123,242},{70,125,244},{70,128,246},{70,130,248},{70,133,250},{70,135,251},{69,138,252},{69,140,253},{68,143,254},{67,145,254},{66,148,255},{65,150,255},{64,153,255},{62,155,254},{61,158,254},{59,160,253},{58,163,252},{56,165,251},{55,168,250},{53,171,248},{51,173,247},{49,175,245},{47,178,244},{46,180,242},{44,183,240},{42,185,238},{40,188,235},{39,190,233},{37,192,231},{35,195,228},{34,197,226},{32,199,223},{31,201,221},{30,203,218},{28,205,216},{27,208,213},{26,210,210},{26,212,208},{25,213,205},{24,215,202},{24,217,200},{24,219,197},{24,221,194},{24,222,192},{24,224,189},{25,226,187},{25,227,185},{26,228,182},{28,230,180},{29,231,178},{31,233,175},{32,234,172},{34,235,170},{37,236,167},{39,238,164},{42,239,161},{44,240,158},{47,241,155},{50,242,152},{53,243,148},{56,244,145},{60,245,142},{63,246,138},{67,247,135},{70,248,132},{74,248,128},{78,249,125},{82,250,122},{85,250,118},{89,251,115},{93,252,111},{97,252,108},{101,253,105},{105,253,102},{109,254,98},{113,254,95},{117,254,92},{121,254,89},{125,255,86},{128,255,83},{132,255,81},{136,255,78},{139,255,75},{143,255,73},{146,255,71},{150,254,68},{153,254,66},{156,254,64},{159,253,63},{161,253,61},{164,252,60},{167,252,58},{169,251,57},{172,251,56},{175,250,55},{177,249,54},{180,248,54},{183,247,53},{185,246,53},{188,245,52},{190,244,52},{193,243,52},{195,241,52},{198,240,52},{200,239,52},{203,237,52},{205,236,52},{208,234,52},{210,233,53},{212,231,53},{215,229,53},{217,228,54},{219,226,54},{221,224,55},{223,223,55},{225,221,55},{227,219,56},{229,217,56},{231,215,57},{233,213,57},{235,211,57},{236,209,58},{238,207,58},{239,205,58},{241,203,58},{242,201,58},{244,199,58},{245,197,58},{246,195,58},{247,193,58},{248,190,57},{249,188,57},{250,186,57},{251,184,56},{251,182,55},{252,179,54},{252,177,54},{253,174,53},{253,172,52},{254,169,51},{254,167,50},{254,164,49},{254,161,48},{254,158,47},{254,155,45},{254,153,44},{254,150,43},{254,147,42},{254,144,41},{253,141,39},{253,138,38},{252,135,37},{252,132,35},{251,129,34},{251,126,33},{250,123,31},{249,120,30},{249,117,29},{248,114,28},{247,111,26},{246,108,25},{245,105,24},{244,102,23},{243,99,21},{242,96,20},{241,93,19},{240,91,18},{239,88,17},{237,85,16},{236,83,15},{235,80,14},{234,78,13},{232,75,12},{231,73,12},{229,71,11},{228,69,10},{226,67,10},{225,65,9},{223,63,8},{221,61,8},{220,59,7},{218,57,7},{216,55,6},{214,53,6},{212,51,5},{210,49,5},{208,47,5},{206,45,4},{204,43,4},{202,42,4},{200,40,3},{197,38,3},{195,37,3},{193,35,2},{190,33,2},{188,32,2},{185,30,2},{183,29,2},{180,27,1},{178,26,1},{175,24,1},{172,23,1},{169,22,1},{167,20,1},{164,19,1},{161,18,1},{158,16,1},{155,15,1},{152,14,1},{149,13,1},{146,11,1},{142,10,1},{139,9,2},{136,8,2},{133,7,2},{129,6,2},{126,5,2},{122,4,3}};


constexpr float viridis_srgb_floats[256][3] = {{0.267004,0.004874,0.329415},{0.268510,0.009605,0.335427},{0.269944,0.014625,0.341379},{0.271305,0.019942,0.347269},{0.272594,0.025563,0.353093},{0.273809,0.031497,0.358853},{0.274952,0.037752,0.364543},{0.276022,0.044167,0.370164},{0.277018,0.050344,0.375715},{0.277941,0.056324,0.381191},{0.278791,0.062145,0.386592},{0.279566,0.067836,0.391917},{0.280267,0.073417,0.397163},{0.280894,0.078907,0.402329},{0.281446,0.084320,0.407414},{0.281924,0.089666,0.412415},{0.282327,0.094955,0.417331},{0.282656,0.100196,0.422160},{0.282910,0.105393,0.426902},{0.283091,0.110553,0.431554},{0.283197,0.115680,0.436115},{0.283229,0.120777,0.440584},{0.283187,0.125848,0.444960},{0.283072,0.130895,0.449241},{0.282884,0.135920,0.453427},{0.282623,0.140926,0.457517},{0.282290,0.145912,0.461510},{0.281887,0.150881,0.465405},{0.281412,0.155834,0.469201},{0.280868,0.160771,0.472899},{0.280255,0.165693,0.476498},{0.279574,0.170599,0.479997},{0.278826,0.175490,0.483397},{0.278012,0.180367,0.486697},{0.277134,0.185228,0.489898},{0.276194,0.190074,0.493001},{0.275191,0.194905,0.496005},{0.274128,0.199721,0.498911},{0.273006,0.204520,0.501721},{0.271828,0.209303,0.504434},{0.270595,0.214069,0.507052},{0.269308,0.218818,0.509577},{0.267968,0.223549,0.512008},{0.266580,0.228262,0.514349},{0.265145,0.232956,0.516599},{0.263663,0.237631,0.518762},{0.262138,0.242286,0.520837},{0.260571,0.246922,0.522828},{0.258965,0.251537,0.524736},{0.257322,0.256130,0.526563},{0.255645,0.260703,0.528312},{0.253935,0.265254,0.529983},{0.252194,0.269783,0.531579},{0.250425,0.274290,0.533103},{0.248629,0.278775,0.534556},{0.246811,0.283237,0.535941},{0.244972,0.287675,0.537260},{0.243113,0.292092,0.538516},{0.241237,0.296485,0.539709},{0.239346,0.300855,0.540844},{0.237441,0.305202,0.541921},{0.235526,0.309527,0.542944},{0.233603,0.313828,0.543914},{0.231674,0.318106,0.544834},{0.229739,0.322361,0.545706},{0.227802,0.326594,0.546532},{0.225863,0.330805,0.547314},{0.223925,0.334994,0.548053},{0.221989,0.339161,0.548752},{0.220057,0.343307,0.549413},{0.218130,0.347432,0.550038},{0.216210,0.351535,0.550627},{0.214298,0.355619,0.551184},{0.212395,0.359683,0.551710},{0.210503,0.363727,0.552206},{0.208623,0.367752,0.552675},{0.206756,0.371758,0.553117},{0.204903,0.375746,0.553533},{0.203063,0.379716,0.553925},{0.201239,0.383670,0.554294},{0.199430,0.387607,0.554642},{0.197636,0.391528,0.554969},{0.195860,0.395433,0.555276},{0.194100,0.399323,0.555565},{0.192357,0.403199,0.555836},{0.190631,0.407061,0.556089},{0.188923,0.410910,0.556326},{0.187231,0.414746,0.556547},{0.185556,0.418570,0.556753},{0.183898,0.422383,0.556944},{0.182256,0.426184,0.557120},{0.180629,0.429975,0.557282},{0.179019,0.433756,0.557430},{0.177423,0.437527,0.557565},{0.175841,0.441290,0.557685},{0.174274,0.445044,0.557792},{0.172719,0.448791,0.557885},{0.171176,0.452530,0.557965},{0.169646,0.456262,0.558030},{0.168126,0.459988,0.558082},{0.166617,0.463708,0.558119},{0.165117,0.467423,0.558141},{0.163625,0.471133,0.558148},{0.162142,0.474838,0.558140},{0.160665,0.478540,0.558115},{0.159194,0.482237,0.558073},{0.157729,0.485932,0.558013},{0.156270,0.489624,0.557936},{0.154815,0.493313,0.557840},{0.153364,0.497000,0.557724},{0.151918,0.500685,0.557587},{0.150476,0.504369,0.557430},{0.149039,0.508051,0.557250},{0.147607,0.511733,0.557049},{0.146180,0.515413,0.556823},{0.144759,0.519093,0.556572},{0.143343,0.522773,0.556295},{0.141935,0.526453,0.555991},{0.140536,0.530132,0.555659},{0.139147,0.533812,0.555298},{0.137770,0.537492,0.554906},{0.136408,0.541173,0.554483},{0.135066,0.544853,0.554029},{0.133743,0.548535,0.553541},{0.132444,0.552216,0.553018},{0.131172,0.555899,0.552459},{0.129933,0.559582,0.551864},{0.128729,0.563265,0.551229},{0.127568,0.566949,0.550556},{0.126453,0.570633,0.549841},{0.125394,0.574318,0.549086},{0.124395,0.578002,0.548287},{0.123463,0.581687,0.547445},{0.122606,0.585371,0.546557},{0.121831,0.589055,0.545623},{0.121148,0.592739,0.544641},{0.120565,0.596422,0.543611},{0.120092,0.600104,0.542530},{0.119738,0.603785,0.541400},{0.119512,0.607464,0.540218},{0.119423,0.611141,0.538982},{0.119483,0.614817,0.537692},{0.119699,0.618490,0.536347},{0.120081,0.622161,0.534946},{0.120638,0.625828,0.533488},{0.121380,0.629492,0.531973},{0.122312,0.633153,0.530398},{0.123444,0.636809,0.528763},{0.124780,0.640461,0.527068},{0.126326,0.644107,0.525311},{0.128087,0.647749,0.523491},{0.130067,0.651384,0.521608},{0.132268,0.655014,0.519661},{0.134692,0.658636,0.517649},{0.137339,0.662252,0.515571},{0.140210,0.665859,0.513427},{0.143303,0.669459,0.511215},{0.146616,0.673050,0.508936},{0.150148,0.676631,0.506589},{0.153894,0.680203,0.504172},{0.157851,0.683765,0.501686},{0.162016,0.687316,0.499129},{0.166383,0.690856,0.496502},{0.170948,0.694384,0.493803},{0.175707,0.697900,0.491033},{0.180653,0.701402,0.488189},{0.185783,0.704891,0.485273},{0.191090,0.708366,0.482284},{0.196571,0.711827,0.479221},{0.202219,0.715272,0.476084},{0.208030,0.718701,0.472873},{0.214000,0.722114,0.469588},{0.220124,0.725509,0.466226},{0.226397,0.728888,0.462789},{0.232815,0.732247,0.459277},{0.239374,0.735588,0.455688},{0.246070,0.738910,0.452024},{0.252899,0.742211,0.448284},{0.259857,0.745492,0.444467},{0.266941,0.748751,0.440573},{0.274149,0.751988,0.436601},{0.281477,0.755203,0.432552},{0.288921,0.758394,0.428426},{0.296479,0.761561,0.424223},{0.304148,0.764704,0.419943},{0.311925,0.767822,0.415586},{0.319809,0.770914,0.411152},{0.327796,0.773980,0.406640},{0.335885,0.777018,0.402049},{0.344074,0.780029,0.397381},{0.352360,0.783011,0.392636},{0.360741,0.785964,0.387814},{0.369214,0.788888,0.382914},{0.377779,0.791781,0.377939},{0.386433,0.794644,0.372886},{0.395174,0.797475,0.367757},{0.404001,0.800275,0.362552},{0.412913,0.803041,0.357269},{0.421908,0.805774,0.351910},{0.430983,0.808473,0.346476},{0.440137,0.811138,0.340967},{0.449368,0.813768,0.335384},{0.458674,0.816363,0.329727},{0.468053,0.818921,0.323998},{0.477504,0.821444,0.318195},{0.487026,0.823929,0.312321},{0.496615,0.826376,0.306377},{0.506271,0.828786,0.300362},{0.515992,0.831158,0.294279},{0.525776,0.833491,0.288127},{0.535621,0.835785,0.281908},{0.545524,0.838039,0.275626},{0.555484,0.840254,0.269281},{0.565498,0.842430,0.262877},{0.575563,0.844566,0.256415},{0.585678,0.846661,0.249897},{0.595839,0.848717,0.243329},{0.606045,0.850733,0.236712},{0.616293,0.852709,0.230052},{0.626579,0.854645,0.223353},{0.636902,0.856542,0.216620},{0.647257,0.858400,0.209861},{0.657642,0.860219,0.203082},{0.668054,0.861999,0.196293},{0.678489,0.863742,0.189503},{0.688944,0.865448,0.182725},{0.699415,0.867117,0.175971},{0.709898,0.868751,0.169257},{0.720391,0.870350,0.162603},{0.730889,0.871916,0.156029},{0.741388,0.873449,0.149561},{0.751884,0.874951,0.143228},{0.762373,0.876424,0.137064},{0.772852,0.877868,0.131109},{0.783315,0.879285,0.125405},{0.793760,0.880678,0.120005},{0.804182,0.882046,0.114965},{0.814576,0.883393,0.110347},{0.824940,0.884720,0.106217},{0.835270,0.886029,0.102646},{0.845561,0.887322,0.099702},{0.855810,0.888601,0.097452},{0.866013,0.889868,0.095953},{0.876168,0.891125,0.095250},{0.886271,0.892374,0.095374},{0.896320,0.893616,0.096335},{0.906311,0.894855,0.098125},{0.916242,0.896091,0.100717},{0.926106,0.897330,0.104071},{0.935904,0.898570,0.108131},{0.945636,0.899815,0.112838},{0.955300,0.901065,0.118128},{0.964894,0.902323,0.123941},{0.974417,0.903590,0.130215},{0.983868,0.904867,0.136897},{0.993248,0.906157,0.143936}};


//Define colormaps
const Image PLENO_COLORMAP_VIRIDIS = [](void) -> Image {
	Image colormap = Image{1, 256, CV_8UC3};
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3b>(0,i) = 
			cv::Vec3b{
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[i][2]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[i][1]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[i][0])
			};	
	return colormap;
}();

const Image PLENO_COLORMAP_VIRIDIS_INV = [](void) -> Image {
	Image colormap = Image{1, 256, CV_8UC3};
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3b>(0,i) = 
			cv::Vec3b{
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[255 - i][2]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[255 - i][1]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[255 - i][0])
			};	
	return colormap;
}();

const Image PLENO_COLORMAP_VIRIDIS_32F = [](void) -> Image {
	Image colormap = Image{1, 256, CV_32FC3};
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3f>(0,i) = cv::Vec3f{viridis_srgb_floats[i][2], viridis_srgb_floats[i][1], viridis_srgb_floats[i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_VIRIDIS_32F_INV = [](void) -> Image {
	Image colormap = Image{1, 256, CV_32FC3};
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3f>(0,i) = cv::Vec3f{viridis_srgb_floats[255 - i][2], viridis_srgb_floats[255 - i][1], viridis_srgb_floats[255 - i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_TURBO = [](void) -> Image {
	Image colormap = Image{1, 256, CV_8UC3};
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3b>(0,i) = cv::Vec3b{turbo_srgb_bytes[i][2], turbo_srgb_bytes[i][1], turbo_srgb_bytes[i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_TURBO_INV = [](void) -> Image {
	Image colormap = Image{1, 256, CV_8UC3};
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3b>(0,i) = cv::Vec3b{turbo_srgb_bytes[255 - i][2], turbo_srgb_bytes[255 - i][1], turbo_srgb_bytes[255 - i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_TURBO_32F = [](void) -> Image {
	Image colormap = Image{1, 256, CV_32FC3};
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3f>(0,i) = cv::Vec3f{turbo_srgb_floats[i][2], turbo_srgb_floats[i][1], turbo_srgb_floats[i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_TURBO_32F_INV = [](void) -> Image {
	Image colormap = Image{1, 256, CV_32FC3};
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3f>(0,i) = cv::Vec3f{turbo_srgb_floats[255 - i][2], turbo_srgb_floats[255 - i][1], turbo_srgb_floats[255 - i][0]};	
	return colormap;
}();


