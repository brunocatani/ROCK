#include <ROCK/Client.h>
#include <ROCK/Hands.h>

namespace modular_example {
    class HandObserver {
        rock::api::Client _client;
        const rock::api::hands::ApiV1* _hands{};
    public:
        rock::api::Status connect(rock::api::QueryInterfaceV1 query) {
            auto status=_client.connect(query,"Example hand observer");
            if (status!=rock::api::Status::Ok) return status;
            status=_client.acquire(1,_hands);
            if (status!=rock::api::Status::Ok) (void)_client.close();
            return status;
        }
        bool sampleRightHand(rock::api::hands::HandFrameV1& output) {
            output={};
            return _hands && _hands->getHandFrameV1(_client.owner(),rock::api::Hand::Right,&output)==rock::api::Status::Ok;
        }
        rock::api::Status close() {
            const auto status=_client.close();
            if (status==rock::api::Status::Ok) _hands=nullptr;
            return status;
        }
    };
}
