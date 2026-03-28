spawnSidsFunction

function sleep(ms) {
	return new Promise(resolve => setTimeout(resolve,ms))
}


var score = 0
let sidSpawnTime = Math.floor(Math.random()*5000)
let spawnSids = true

async function createScoreText() {
	const scoreText = document.createElement('div')
	scoreText.textContent = 'Score: 0'
	scoreText.zIndex = 2147483647
	scoreText.style.top = '100px'
	scoreText.style.position = 'fixed'
	scoreText.id = 'ScoreText'
	document.body.appendChild(scoreText)
}

async function chooseSid() {
	const sidList = ['sid.png', 'pineapple.png', 'sid2.png', 'sid3.png','sid4.png', 'sid5.png']
	const chosenSid = sidList[Math.floor(Math.random()*sidList.length)]
	pointValue = 1
	if (chosenSid == "pineapple.png") {
		pointValue = -1
	}
	let randomColor = false
	if (Math.random() > 0.75) {
		randomColor = true
	}
	return [chosenSid, pointValue, randomColor]
}

async function createSidButton() {
	const newSidButton = document.createElement('button')
	const newImg = document.createElement('img')
	const chosenSidInfo = await chooseSid()
	const chosenSid = chosenSidInfo[0]
	const pointValue = chosenSidInfo[1]
	const randomColor = chosenSidInfo[2]
	const imgSize = Math.floor(Math.random()*100)
	newImg.src = chrome.runtime.getURL('images/'+chosenSid)
	newSidButton.width = `${imgSize}`
	newSidButton.height = `${imgSize}`
	newImg.width = `${imgSize}`
	newImg.height = `${imgSize}`
	newImg.style.top = '0px'
	newImg.style.filter = `hue-rotate(${Math.floor(Math.random()*360)}deg)`
	newSidButton.style.top = `${Math.floor(Math.random()*document.body.scrollHeight)-(1.5*imgSize)}px`
	newSidButton.style.left = `${Math.floor(Math.random()*document.body.scrollWidth)-(1.5*imgSize)}px`
	newSidButton.style.position = 'absolute'
	newImg.style.position = 'relative'
	newSidButton.bgColor = 'transparent'
	newSidButton.zIndex = 2147483640
	newImg.zIndex = 2147483640
	document.body.appendChild(newSidButton)
	newSidButton.appendChild(newImg)
	newSidButton.addEventListener('click', ()=> {const scoreText = document.getElementById('ScoreText'); score += pointValue; scoreText.textContent = `Score: ${score}`; newSidButton.remove()})
}

async function spawnSidsFunction() {
	while (spawnSids == true) {
		await createSidButton()
		await sleep(sidSpawnTime)
	}
}

createScoreText()
spawnSidsFunction()