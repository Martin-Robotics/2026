

function sleep(ms) {
return new Promise(resolve => setTimeout(resolve,ms))
};

function getActiveTab() {
chrome.tabs.query({active: true, currentWindow: true}, (tabs)=> {
activeTab = tabs[0]
const sidsToSpawn = Number(document.getElementById("sidNumber").value)
const cooldown = Number(document.getElementById("cooldown").value)
const endless = document.getElementById('endless').checked
loopTab(activeTab.id, sidsToSpawn, cooldown, endless)
})
}

async function loopTab(tabID, sidsToSpawn, cooldown, endless) {
while (endless == true) {
chrome.scripting.executeScript({target: {tabId: tabID}, func: createSid})
await sleep(cooldown)
}
while (sidsToSpawn > 0 && endless == false) {
chrome.scripting.executeScript({target: {tabId: tabID}, func: createSid})
sidsToSpawn -=  1
await sleep(cooldown)
}}

function createSid() {
const sidSize = Math.floor(Math.random()*250)
const xPos = Math.floor(Math.random()*(window.innerWidth-(sidSize*1.05)))
const yPos = Math.floor(Math.random()*(document.documentElement.scrollHeight-(sidSize*1.05)))
const newSid = document.createElement('img')

const imageList = ["sid.png", "pineapple.png"]
const chosenImage = imageList[Math.floor(Math.random()*imageList.length)]
newSid.src = chrome.runtime.getURL(chosenImage)
newSid.width = `${sidSize}`
newSid.height = `${sidSize}`
newSid.style.left = `${xPos}px`
newSid.style.top = `${yPos}px`
newSid.style.position = 'absolute'
newSid.zIndex = 1000000000000000
document.body.appendChild(newSid)
}

function initSidTheImages() {
chrome.tabs.query({active: true, currentWindow: true}, (tabs) => {
if (tabs[0]) {
chrome.scripting.executeScript({target: {tabId: tabs[0].id}, function: sidTheImages})
}})}

function sidTheImages() {
const allImages = document.getElementsByTagName('img')
for (const image of allImages) {
image.src = chrome.runtime.getURL('sid.png')
}
}




// we will not have a meeting on monday, tuesday normal, no wednesday, thursday is just drive team, saturday is normal

document.getElementById("spawnButton").addEventListener('click', getActiveTab)
document.getElementById("sidButton").addEventListener('click', initSidTheImages)