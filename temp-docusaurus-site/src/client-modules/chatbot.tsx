
import React from 'react';
import ReactDOM from 'react-dom';
import Chatbot from '../theme/Chatbot';

const ICON_ID = 'rag-chatbot-icon';
const CHATBOT_CONTAINER_ID = 'rag-chatbot-container';

const createChatbotContainer = () => {
    let container = document.getElementById(CHATBOT_CONTAINER_ID);
    if (!container) {
        container = document.createElement('div');
        container.id = CHATBOT_CONTAINER_ID;
        document.body.appendChild(container);
    }
    return container;
};

const showIcon = (x, y, selection) => {
    let icon = document.getElementById(ICON_ID);
    if (!icon) {
        icon = document.createElement('div');
        icon.id = ICON_ID;
        icon.style.position = 'absolute';
        icon.style.cursor = 'pointer';
        icon.innerHTML = '🤖'; // Simple robot icon
        icon.style.fontSize = '24px';
        icon.style.zIndex = '1000';
        document.body.appendChild(icon);

        icon.addEventListener('click', () => {
            const container = createChatbotContainer();
            ReactDOM.render(<Chatbot selectedText={selection} />, container);
            hideIcon();
        });
    }
    icon.style.left = `${x}px`;
    icon.style.top = `${y}px`;
    icon.style.display = 'block';
};

const hideIcon = () => {
    const icon = document.getElementById(ICON_ID);
    if (icon) {
        icon.style.display = 'none';
    }
};

const handleTextSelection = (event) => {
    const selection = window.getSelection().toString().trim();
    if (selection) {
        const range = window.getSelection().getRangeAt(0);
        const rect = range.getBoundingClientRect();
        showIcon(rect.right, rect.top + window.scrollY, selection);
    } else {
        hideIcon();
    }
};

export default function () {
    if (typeof window !== 'undefined') {
        document.addEventListener('mouseup', handleTextSelection);
    }
    return null;
}
