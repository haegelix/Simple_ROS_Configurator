class HELPERS {
    setAsList(list, DOMid) {
        const tsys = document.getElementById(DOMid)
        tsys.innerHTML = ""
        for (let t of list) {
            let c = document.createElement("li")
            c.innerText = t.toString()
            tsys.appendChild(c)
        }
    }
    setAsTable(list, DOMid) {
        const tsys = document.getElementById(DOMid)
        tsys.innerHTML = ""
        for (let t of list) {
            let c = document.createElement("tr")
            for(let tt of t){
                let cc = document.createElement("td")
                cc.innerText = tt.toString()
                c.appendChild(cc)
            }
            tsys.appendChild(c)
        }
    }
}

function goToSearchString(searchString){
    const prot = window.location.protocol
    const host = window.location.host
    const path = window.location.pathname
    window.location.href = prot + "//" + host + "" + path + "?" + searchString
}