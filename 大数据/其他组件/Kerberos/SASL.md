

# SASL

SASL全称Simple Authentication and Security Layer，是一种对API加密编码的框架，为服务提供验证、数据完整性检查和加密的能力。

从身份验证的角度来看，SASL是一种用来Server-Server，Server-Client之间进行身份验证的框架。通过SASL，可以避免身份验证时密码等关键信息在网络中明文传输。

在SASL框架中，可以选着以下机制：
|机制|说明|
|---|---|
|EXTERNAL   | 认证信息在内容中(例如已经使用IPsec或传输层安全的协议)
|ANONYMOUS  | 对与未认证的客人的访问
|PLAIN      | 一个简单明文密码机制。PLAIN取代了LOGIN 机制。
|OTP        | 一个临时密码机制。 OTP取代了SKEY机制。
|SKEY       |一个S/KEY机制
|CRAM-MD5   |一个简单的基于HMAC-MD5的询问应答机制。
|DIGEST-MD5 | 是一个HTTP兼容的，基于MD5的询问应答机制。DIGEST-MD5提供了数据层安全。
|NTLM       |一个 NT LAN Manager认证机制。
|GSSAPI     |通过通用安全服务应用程序层的Kerberos V5 协议的安全认证。GSSAPI 提供了数据安全层。
|GateKeeper | Microsoft为Windows Live Messenger开发的一个询问应答机制。


# 组件的应用

Hadoop生态的大部分组件而言，均支持SASL框架提供验证服务。而在启用SASL时，GSSAPI（Kerberos）机制又是首选。

PS：部分组件在WebUI支持用户认证时，使用的是SPNEGO，该协议也是基于GSS-API认证机制的安全协议，可以认为是Kerberos的一种扩展。