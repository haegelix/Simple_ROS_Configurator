//const api = new API()
const helpers = new HELPERS()

function loadTopics(){
    api.getTopics().then(result => {
        //helpers.setAsList(result.sys, "topics_sys")
        //helpers.setAsList(result.user, "topics_user")
        helpers.setAsTable(result.sys, "topics_sys_t")
        helpers.setAsTable(result.user, "topics_user_t")
    })
}

loadTopics()