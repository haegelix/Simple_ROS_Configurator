const prot = window.location.protocol
const host = window.location.host
const apipath = prot + "//" + host + "/api/"

class API {
    getTopics(){
        return axios.get(apipath + "topics")
            .then(result => {
                return Promise.resolve(result.data)
            })
    }
}
