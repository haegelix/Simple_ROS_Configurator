const prot = window.location.protocol
const host = window.location.host
const apipath = prot + "//" + host + "/api/"

const ax = axios.create(
    {baseURL: apipath}
)

class API {

}



const api = new API()