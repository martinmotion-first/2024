function getBlueAllianceTeamData() {
    const allDataContainingRange = "A2:J100";
    //const stLouisEventKey = "2023mosl";
    const stLouisEventKey = "2024mosl";
    //const peoriaEventKey = "2023ilpe";
    const peoriaEventKey = "2024ilpe";
  
    var stLouisTeamsUrl = "https://www.thebluealliance.com/api/v3/event/" + stLouisEventKey + "/teams";  //st louis
    var stLouisSpreadsheetKey = "St Louis";
    var stLouisSheet = SpreadsheetApp.getActive().getSheetByName(stLouisSpreadsheetKey);
    var stLouisRankingsDataUrl = "https://www.thebluealliance.com/api/v3/event/" + stLouisEventKey + "/rankings";
  
    var peoriaTeamsUrl = "https://www.thebluealliance.com/api/v3/event/" + peoriaEventKey + "/teams"; //peoria
    var peoriaSpreadsheetKey = "Peoria";
    var peoriaSheet = SpreadsheetApp.getActive().getSheetByName(peoriaSpreadsheetKey);
    var peoriaRankingsDataUrl = "https://www.thebluealliance.com/api/v3/event/" + peoriaEventKey + "/rankings";
  
    var options = {
      'method' : 'get',
      'contentType': 'application/json',
      'headers': {
          'X-TBA-Auth-Key': '**** NOPE - AVAILABLE ELSEWHERE ****'
        }
    };
  
    loadTeamDataForEvent(stLouisSpreadsheetKey, stLouisSheet, stLouisTeamsUrl, options);
    loadTeamDataForEvent(peoriaSpreadsheetKey, peoriaSheet, peoriaTeamsUrl, options);
  
    loadRankingsDataForEvent(stLouisSpreadsheetKey, stLouisSheet, stLouisRankingsDataUrl, options);
    loadRankingsDataForEvent(peoriaSpreadsheetKey, peoriaSheet, peoriaRankingsDataUrl, options);
  
    sortSheet(peoriaSheet, allDataContainingRange);
    sortSheet(stLouisSheet, allDataContainingRange);
    //clearItAll([stLouisSheet, peoriaSheet], allDataContainingRange);
  }
  
  function clearItAll(sheets, rangeDefinition){
    for(var i = 0; i < sheets.length; i++){
      sheets[i].getRange(rangeDefinition).setValue('');
    }
  }
  
  function sortSheet(spreadsheet, rangeDefinition){
    spreadsheet.getRange(rangeDefinition).sort([
      {column: 5, ascending: true},
      {column: 1, ascending: true}
    ]);
  }
  
  function loadRankingsDataForEvent(key, spreadsheet, url, options){
    var rankingsResponse = UrlFetchApp.fetch(url, options);
    var rankingsParsed = JSON.parse(rankingsResponse).rankings;
    Logger.log(key + " rankings data:" + rankingsParsed);
    var lastRow = spreadsheet.getLastRow();
    var teamKeys = spreadsheet.getRange('D2:D' + lastRow).getValues();
    for(var i = 0; i < teamKeys.length; i++){
      var key = teamKeys[i];
      //Logger.log('i=' + i);
      //Logger.log('key=' + key);
      if(!key || !key.toString().trim()){
        Logger.log("forced continue");
        continue;
      }
      var node = rankingsParsed.filter(r => r.team_key == key);
      if(!node || node.length < 1){
        Logger.log("forced continue2");
        Logger.log("(from key:" + key +")");
        continue;
      }
      node = node[0];
      var cellKey = i + 2;
      spreadsheet.getRange("E" + cellKey).setValue(node.rank);
      spreadsheet.getRange("F" + cellKey).setValue(node.record.wins);
    }
  }
  
  function loadTeamDataForEvent(key, spreadsheet, url, options){
    var responseData = UrlFetchApp.fetch(url, options);
    var parsedData = JSON.parse(responseData);
    Logger.log(key + " team data:" + parsedData);
    for(var i = 0; i < parsedData.length; i++){
      var rowNumber = i + 2;
      spreadsheet.getRange('A' + rowNumber).setValue(parsedData[i].nickname);
      spreadsheet.getRange('B' + rowNumber).setValue(parsedData[i].team_number);
      spreadsheet.getRange('C' + rowNumber).setValue(parsedData[i].rookie_year);
      spreadsheet.getRange('D' + rowNumber).setValue(parsedData[i].key);
    }
  }
  